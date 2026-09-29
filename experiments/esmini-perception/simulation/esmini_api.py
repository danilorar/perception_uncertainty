"""Shared ctypes bindings, ideal sensor access and viewer setup for esmini 3.8.1.

Author: Zhuo Ma
The ABI uses double-precision coordinates. Changing State field order or types
without checking the matching esminiLib.hpp can corrupt the returned data.
"""
import ctypes as C
import math
import xml.etree.ElementTree as ET

from paths import DATA, ESMINI, library_path


class State(C.Structure):
    """SE_ScenarioObjectState: simulation truth, not a sensor measurement packet."""

    _fields_ = [
        ("id", C.c_int), ("model_id", C.c_int), ("ctrl_type", C.c_int),
        ("timestamp", C.c_double),
        # World position [m] and orientation [rad]. Position is a reference point,
        # which can differ from the bounding-box centre used for gap calculation.
        ("x", C.c_double), ("y", C.c_double), ("z", C.c_double),
        ("h", C.c_double), ("p", C.c_double), ("r", C.c_double),
        ("roadId", C.c_uint32), ("junctionId", C.c_uint32),
        ("t", C.c_double), ("laneId", C.c_int),
        ("laneOffset", C.c_double), ("s", C.c_double),
        ("speed", C.c_double),  # m/s, not km/h
        # Bounding-box centre offsets and dimensions [m].
        ("centerOffsetX", C.c_double), ("centerOffsetY", C.c_double),
        ("centerOffsetZ", C.c_double), ("width", C.c_double),
        ("length", C.c_double), ("height", C.c_double),
        # Integer enums: interpret category in the context of its main type.
        ("objectType", C.c_int), ("objectCategory", C.c_int),
        ("wheel_angle", C.c_double), ("wheel_rot", C.c_double),
        ("visibilityMask", C.c_int),
    ]


def load_library():
    """Load the installed native library and declare each function's C signature."""
    path = library_path()
    if not path.is_file():
        raise FileNotFoundError(f"esmini library not found: {path}. Set ESMINI_HOME to the demo directory.")
    se = C.CDLL(str(path))
    # argtypes/restype are necessary: ctypes otherwise assumes C int returns and
    # cannot safely infer pointers or double-precision parameters.
    signatures = {
        "SE_InitWithArgs": (C.c_int, [C.c_int, C.POINTER(C.c_char_p)]),
        "SE_GetIdByName": (C.c_int, [C.c_char_p]),
        "SE_GetQuitFlag": (C.c_int, []),
        "SE_GetSimulationTime": (C.c_double, []),
        "SE_StepDT": (C.c_int, [C.c_double]),
        "SE_AddObjectSensor": (C.c_int, [C.c_int] + [C.c_double] * 7 + [C.c_int]),
        "SE_FetchSensorObjectList": (C.c_int, [C.c_int, C.POINTER(C.c_int)]),
        "SE_GetObjectState": (C.c_int, [C.c_int, C.POINTER(State)]),
        "SE_ReportObjectPos": (C.c_int, [C.c_int] + [C.c_double] * 6),
        "SE_ReportObjectSpeed": (C.c_int, [C.c_int, C.c_double]),
        "SE_GetObjectNumberOfCollisions": (C.c_int, [C.c_int]),
        "SE_ViewerShowFeature": (None, [C.c_int, C.c_bool]),
        "SE_AddCustomFixedCamera": (C.c_int, [C.c_double] * 5),
        "SE_SetCameraMode": (C.c_int, [C.c_int]),
        "SE_Close": (None, []),
        "SE_CloseLogFile": (None, []),
    }
    for name, (return_type, argument_types) in signatures.items():
        function = getattr(se, name)
        function.restype = return_type
        function.argtypes = argument_types
    return se


def get_state(se, object_id):
    """Read truth even if the object is outside the sensor or has been dropped."""
    state = State()
    if se.SE_GetObjectState(object_id, C.byref(state)) != 0:
        raise RuntimeError(f"Cannot read object {object_id}")
    return state


def add_sensor(se, host_id, max_objects=10):
    """Attach the project's ideal forward sensor; return its handle and ID buffer."""
    sensor_id = se.SE_AddObjectSensor(
        host_id,
        2.5, 0.0, 0.5,          # sensor position relative to ego: x/y/z [m]
        0.0,                     # yaw relative to ego [rad]
        0.1, 100.0,             # near/far range [m]
        math.radians(60.0),      # horizontal field of view [rad]
        max_objects,            # maximum number of returned object IDs
    )
    if sensor_id < 0:
        raise RuntimeError("Failed to create ideal object sensor")
    return sensor_id, (C.c_int * max_objects)()


def fetch_ids(se, sensor_id, buffer):
    """Fetch count + object IDs. Position/size/category are NOT returned here."""
    count = se.SE_FetchSensorObjectList(sensor_id, buffer)
    if not 0 <= count <= len(buffer):
        raise RuntimeError(f"Invalid sensor object count: {count}")
    # esmini writes into the supplied C array. Entries after count may be stale.
    return [buffer[index] for index in range(count)]


def scenario_arguments(scenario, output_dir, dt, headless=False, replay=False):
    """Build shared esmini options; disable source controllers only for replay."""
    arguments = [
        "tme180-simulation", "--osc", str(scenario),
        "--path", str(DATA), str(ESMINI / "resources"),
        "--traj_filter", "0", "--seed", "0", "--collision",
        "--fixed_timestep", str(dt),
        "--record", str(output_dir / "sim.dat"),
        "--logfile_path", str(output_dir / "run.log"),
    ]
    if replay:
        arguments += ["--disable_controllers"]
    if headless:
        arguments += ["--headless"]
    else:
        # Keep the established macOS viewer settings and fixed overhead camera.
        arguments += ["--window", "80", "80", "1100", "650", "--SingleThreaded",
                      "--generate_without_textures", "--ground_plane", "off",
                      "--disable_shadows", "--hide_trajectories"]
    return arguments


def initialize(se, arguments):
    """Encode Python strings into the argc/argv interface expected by esmini."""
    argv = (C.c_char_p * len(arguments))(*[item.encode("utf-8") for item in arguments])
    if se.SE_InitWithArgs(len(arguments), argv) != 0:
        raise RuntimeError("Failed to initialize esmini; inspect run.log")


def set_overview_camera(se, source_scenario):
    """Fit BOTH original trajectories; the generated AEB file lacks the ego path."""
    root = ET.parse(source_scenario).getroot()
    positions = root.findall(
        ".//FollowTrajectoryAction/Trajectory/Shape/Polyline/Vertex/Position/WorldPosition"
    )
    if not positions:
        raise ValueError("No original trajectory positions for the overview camera")
    heading = float(positions[0].get("h", "0"))
    cosine, sine = math.cos(heading), math.sin(heading)
    xs = [float(position.get("x")) for position in positions]
    ys = [float(position.get("y")) for position in positions]
    zs = [float(position.get("z", "0")) for position in positions]
    along = [x * cosine + y * sine for x, y in zip(xs, ys)]
    across = [-x * sine + y * cosine for x, y in zip(xs, ys)]
    middle_along = (min(along) + max(along)) / 2
    middle_across = (min(across) + max(across)) / 2
    camera_x = middle_along * cosine - middle_across * sine
    camera_y = middle_along * sine + middle_across * cosine
    height = max((max(along) - min(along) + 24.0) / 0.85,
                 (max(across) - min(across) + 20.0) / 0.50, 60.0)
    camera_id = se.SE_AddCustomFixedCamera(
        camera_x, camera_y, max(zs) + height, heading + math.pi / 2, math.pi / 2
    )
    if camera_id < 0 or se.SE_SetCameraMode(camera_id) != 0:
        raise RuntimeError("Failed to select the fixed overview camera")
