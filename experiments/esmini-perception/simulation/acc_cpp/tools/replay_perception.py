#!/usr/bin/env python3
"""Replay recorded truth in the native viewer; show the cone from logged observations.

Author: Zhuo Ma
No controller or error model is executed. Requires the graphics-enabled esmini
v3.8.1 library, not acc_cpp/build/engine-home's headless library. Python stdlib only.
"""
import argparse
from bisect import bisect_right
import csv
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time
import xml.etree.ElementTree as ET
import zlib

SENSOR_FEATURE = 1  # roadgeom::NODE_MASK_OBJECT_SENSORS in esmini v3.8.1
EPS = 1e-8


class State(C.Structure):
    """SE_ScenarioObjectState: simulation truth, not a sensor measurement packet.

    The ABI uses double-precision coordinates. Changing field order or types
    without checking the matching esminiLib.hpp can corrupt the returned data.
    """

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


class Image(C.Structure):
    _fields_ = [("width", C.c_int), ("height", C.c_int), ("pixelSize", C.c_int),
                ("pixelFormat", C.c_int), ("data", C.POINTER(C.c_ubyte))]


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_recording(run):
    config = json.loads((run / "run_config.json").read_text())
    if config.get("esmini_tag") != "v3.8.1":
        raise ValueError("This replay's ctypes ABI requires an esmini v3.8.1 recording/library.")
    # Runs before this option existed always wrote detailed logs.
    if not config.get("detailed_logs", True):
        raise ValueError("Replay needs truth/perception/control logs; rerun acc_sim with --detailed-logs true.")
    truth = {}
    for row in rows(run / "truth.csv"):
        truth.setdefault(float(row["time_s"]), []).append(row)
    frames = sorted(truth.items())
    if not frames or abs(frames[0][0]) > EPS:
        raise ValueError("Recording must include truth at t=0.")
    ids = {int(row["object_id"]) for row in frames[0][1]}
    if len(ids) != 2 or any({int(r["object_id"]) for r in f} != ids for _, f in frames):
        raise ValueError("This utility supports the experiment's two-vehicle recordings.")
    controls = rows(run / "control.csv")
    if not controls:
        raise ValueError("control.csv has no recorded perception sequence.")
    control_times = [float(row["time_s"]) for row in controls]
    if control_times != sorted(control_times) or abs(control_times[0]) > EPS:
        raise ValueError("Control timestamps must be ordered and start at t=0.")
    perceptions = {}
    for row in rows(run / "perceptions.csv"):
        perceptions.setdefault(int(row["sequence"]), []).append(row)
    return config, frames, controls, control_times, perceptions


def displayed_observation(t, controls, control_times, perceptions):
    index = bisect_right(control_times, t + EPS) - 1
    control = controls[index]
    sequence = int(control["perception_sequence"])
    observation = perceptions.get(sequence, [])
    observed = [int(r["track_id"]) for r in observation if r["observed"] == "1"]
    dropped = [int(r["track_id"]) for r in observation if r["dropped"] == "1"]
    status = "DETECTED" if observed else "DROPOUT" if dropped else "NO_DETECTION"
    return sequence, bool(observed), status, control["aeb_active"] == "1"


def make_scene(config, first_frame, destination):
    source = Path(config["generated_scenario"])
    tree = ET.parse(source)
    root = tree.getroot()
    road = root.find("./RoadNetwork/LogicFile")
    road_path = Path(road.get("filepath"))
    if not road_path.is_absolute():
        road_path = source.parent / road_path
    if not road_path.is_file():
        raise FileNotFoundError(road_path)
    road.set("filepath", str(road_path.resolve()))
    objects = root.findall("./Entities/ScenarioObject")
    if len(objects) != len(first_frame):
        raise ValueError("Generated XOSC and truth have different object counts.")
    root.remove(root.find("Storyboard"))
    board = ET.SubElement(root, "Storyboard")
    actions = ET.SubElement(ET.SubElement(board, "Init"), "Actions")
    for obj, row in zip(objects, sorted(first_frame, key=lambda r: int(r["object_id"]))):
        old = obj.find("ObjectController")
        if old is not None:
            obj.remove(old)
        controller = ET.SubElement(ET.SubElement(obj, "ObjectController"), "Controller",
                                   name="RecordedTruthReplay")
        properties = ET.SubElement(controller, "Properties")
        for name, value in (("esminiController", "ExternalController"), ("mode", "override")):
            ET.SubElement(properties, "Property", name=name, value=value)
        private = ET.SubElement(actions, "Private", entityRef=obj.get("name"))
        pose = ET.SubElement(ET.SubElement(ET.SubElement(private, "PrivateAction"),
                                          "TeleportAction"), "Position")
        ET.SubElement(pose, "WorldPosition", x=row["x_m"], y=row["y_m"],
                      z=row["z_m"], h=row["h_rad"], p="0", r="0")
        activation = ET.SubElement(ET.SubElement(private, "PrivateAction"), "ControllerAction")
        ET.SubElement(activation, "ActivateControllerAction", longitudinal="true", lateral="true")
    ET.SubElement(board, "StopTrigger")
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def checked(result, operation):
    if result != 0:
        raise RuntimeError(f"{operation} failed: {result}")


def bind(home):
    filename = {"darwin": "libesminiLib.dylib", "win32": "esminiLib.dll"}.get(sys.platform, "libesminiLib.so")
    path = home / "bin" / filename
    if not path.is_file():
        raise FileNotFoundError(f"Graphics-enabled library not found: {path}")
    lib = C.CDLL(str(path))
    signatures = {
        "SE_InitWithArgs": (C.c_int, [C.c_int, C.POINTER(C.c_char_p)]),
        "SE_GetNumberOfObjects": (C.c_int, []),
        "SE_GetIdByName": (C.c_int, [C.c_char_p]),
        "SE_GetSimulationTime": (C.c_double, []),
        "SE_GetQuitFlag": (C.c_int, []),
        "SE_GetObjectState": (C.c_int, [C.c_int, C.POINTER(State)]),
        "SE_ReportObjectPos": (C.c_int, [C.c_int] + [C.c_double] * 6),
        "SE_ReportObjectSpeed": (C.c_int, [C.c_int, C.c_double]),
        "SE_ReportObjectVel": (C.c_int, [C.c_int] + [C.c_double] * 3),
        "SE_ReportObjectAcc": (C.c_int, [C.c_int] + [C.c_double] * 3),
        "SE_StepDT": (C.c_int, [C.c_double]),
        "SE_AddObjectSensor": (C.c_int, [C.c_int] + [C.c_double] * 7 + [C.c_int]),
        "SE_ViewerShowFeature": (None, [C.c_int, C.c_bool]),
        "SE_SetCameraObjectFocus": (C.c_int, [C.c_int]),
        "SE_AddCustomCamera": (C.c_int, [C.c_double] * 5),
        "SE_SetCameraMode": (C.c_int, [C.c_int]),
        "SE_SaveImagesToRAM": (C.c_int, [C.c_bool]),
        "SE_FetchImage": (C.c_int, [C.POINTER(Image)]),
        "SE_Close": (None, []),
        "SE_CloseLogFile": (None, []),
    }
    for name, (restype, argtypes) in signatures.items():
        function = getattr(lib, name)
        function.restype, function.argtypes = restype, argtypes
    return lib


def report(lib, frame):
    for row in frame:
        oid = int(row["object_id"])
        checked(lib.SE_ReportObjectPos(oid, *[float(row[k]) for k in ("x_m", "y_m", "z_m", "h_rad")], 0, 0), "Report pose")
        checked(lib.SE_ReportObjectSpeed(oid, float(row["speed_mps"])), "Report speed")
        for function, columns in ((lib.SE_ReportObjectVel, ("vx_mps", "vy_mps", "vz_mps")),
                                  (lib.SE_ReportObjectAcc, ("ax_mps2", "ay_mps2", "az_mps2"))):
            checked(function(oid, *[float(row[k]) for k in columns]), "Report motion")


def verify(lib, t, frame):
    error = abs(lib.SE_GetSimulationTime() - t)
    for row in frame:
        state = State()
        checked(lib.SE_GetObjectState(int(row["object_id"]), C.byref(state)), "Read replay state")
        if state.ctrl_type != 1:
            raise RuntimeError("Every replay vehicle must use ExternalController.")
        for attribute, column in (("x", "x_m"), ("y", "y_m"), ("z", "z_m"),
                                  ("h", "h_rad"), ("speed", "speed_mps")):
            error = max(error, abs(getattr(state, attribute) - float(row[column])))
    if error > 1e-6:
        raise RuntimeError(f"Replay differs from recorded truth at t={t:.2f}: {error}")
    return error


def save_png(lib, path):
    """Save native RGB/BGR framebuffer as PNG, without a Pillow dependency."""
    frame = Image()
    checked(lib.SE_FetchImage(C.byref(frame)), "Fetch rendered image")
    if not frame.data or frame.pixelSize != 3 or frame.pixelFormat not in (0x1907, 0x80E0):
        raise RuntimeError("Native viewer did not return an RGB/BGR image.")
    raw = C.string_at(frame.data, frame.width * frame.height * 3)
    if frame.pixelFormat == 0x80E0:
        rgb = bytearray(raw)
        rgb[0::3], rgb[2::3] = raw[2::3], raw[0::3]
        raw = bytes(rgb)
    stride = frame.width * 3
    scanlines = b"".join(b"\0" + raw[y * stride:(y + 1) * stride]
                          for y in reversed(range(frame.height)))
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">2I5B", frame.width, frame.height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(scanlines)) + chunk(b"IEND", b""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True, help="One ideal_seed*/dropout_seed* directory")
    parser.add_argument("--esmini-home", type=Path, default=Path(__file__).resolve().parents[5] / "esmini")
    parser.add_argument("--speed", type=float, default=0.5, help="Playback speed; .5 is half speed")
    parser.add_argument("--camera", choices=("top", "rear"), default="top")
    parser.add_argument("--hold-end", type=float, default=3, help="Seconds to retain final frame")
    parser.add_argument("--no-realtime", action="store_true", help="Render/verify as fast as possible")
    parser.add_argument("--check-only", action="store_true", help="Verify all recorded states without opening a window")
    parser.add_argument("--snapshots", type=float, nargs="*", default=[], help="Recorded times to capture as PNG")
    parser.add_argument("--output-dir", type=Path, help="Default: RUN/sensor_replay")
    args = parser.parse_args()
    if args.speed <= 0 or args.hold_end < 0 or (args.check_only and args.snapshots):
        parser.error("Require speed > 0, hold-end >= 0; snapshots need the viewer.")
    run, home = args.run_dir.resolve(), args.esmini_home.resolve()
    config, frames, controls, control_times, perceptions = load_recording(run)
    output = args.output_dir.resolve() if args.output_dir else run / "sensor_replay"
    if output == run:
        parser.error("Use a separate output directory, not the input run directory.")
    snapshots = {min(range(len(frames)), key=lambda i: abs(frames[i][0] - requested)) for requested in args.snapshots}
    if any(requested < 0 or requested > frames[-1][0] + EPS for requested in args.snapshots):
        parser.error("Snapshot time lies outside this recording.")
    inputs = [run / name for name in ("truth.csv", "perceptions.csv", "control.csv", "run_config.json", "summary.json")]
    before = {str(path): digest(path) for path in inputs}
    output.mkdir(parents=True, exist_ok=True)
    scene = output / "visual_replay.xosc"
    make_scene(config, frames[0][1], scene)
    lib = bind(home)
    options = ["perception_replay", "--osc", str(scene), "--path", str(home / "resources"),
               "--SingleThreaded", "--fixed_timestep", str(config["dt_s"]), "--traj_filter", "0",
               "--disable_stdout", "--logfile_path", str(output / "viewer.log")]
    options += ["--headless"] if args.check_only else ["--window", "60", "60", "1100", "700", "--info_text", "0", "--disable_shadows"]
    argv = (C.c_char_p * len(options))(*[option.encode() for option in options])
    checked_count, maximum_error, previous_status = 0, 0, None
    captured, transitions = [], []
    try:
        checked(lib.SE_InitWithArgs(len(options), argv), "Initialize native replay")
        ego_id = lib.SE_GetIdByName(b"object_1")
        if ego_id < 0 or lib.SE_GetNumberOfObjects() != 2:
            raise RuntimeError("Expected object_1 and one target.")
        if lib.SE_AddObjectSensor(ego_id, 2.5, 0, .5, 0, .1, config["sensor_far_m"], config["sensor_fov_rad"], 64) < 0:
            raise RuntimeError("Cannot create the display sensor.")
        if not args.check_only:
            checked(lib.SE_SetCameraObjectFocus(ego_id), "Focus ego")
            # Frame both cars and the full 60 m frustum, including space behind ego.
            pose = (30, 0, 170, 0, math.pi / 2) if args.camera == "top" else (-25, -25, 30, .65, .65)
            camera = lib.SE_AddCustomCamera(*pose)
            if camera < 0:
                raise RuntimeError("Viewer unavailable: use the graphics-enabled esmini library.")
            checked(lib.SE_SetCameraMode(camera), "Select camera")
            checked(lib.SE_SaveImagesToRAM(bool(snapshots)), "Configure image capture")
        started = time.monotonic()
        with (output / "cone_visibility.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "perception_sequence", "cone_visible", "status", "aeb_active"])
            previous_time = 0.0
            for index, (t, frame) in enumerate(frames):
                if lib.SE_GetQuitFlag():
                    break
                sequence, visible, status, active = displayed_observation(t, controls, control_times, perceptions)
                lib.SE_ViewerShowFeature(SENSOR_FEATURE, visible)
                report(lib, frame)
                checked(lib.SE_StepDT(t - previous_time), "Advance visual replay")
                maximum_error = max(maximum_error, verify(lib, t, frame))
                checked_count += 1
                writer.writerow([f"{t:.8f}", sequence, int(visible), status, int(active)])
                if (visible, status) != previous_status:
                    print(f"t={t:.2f}s  cone={'ON' if visible else 'OFF'}  {status}", flush=True)
                    transitions.append({"time_s": t, "visible": visible, "status": status})
                    previous_status = (visible, status)
                if index in snapshots:
                    path = output / "snapshots" / f"t_{t:.2f}_cone_{'on' if visible else 'off'}.png"
                    save_png(lib, path)
                    captured.append(str(path))
                previous_time = t
                if not args.check_only and not args.no_realtime:
                    time.sleep(max(0, started + t / args.speed - time.monotonic()))
            if not args.check_only and not args.no_realtime:
                deadline = time.monotonic() + args.hold_end
                while time.monotonic() < deadline and not lib.SE_GetQuitFlag():
                    checked(lib.SE_StepDT(0), "Keep final frame")
                    time.sleep(.02)
    finally:
        lib.SE_Close()
        lib.SE_CloseLogFile()
    if {str(path): digest(path) for path in inputs} != before:
        raise RuntimeError("An original experiment input changed during replay.")
    if len(captured) != len(snapshots):
        raise RuntimeError("Window closed before all requested snapshots were captured.")
    summary = {"source_run": str(run), "source_sha256": before, "viewer": not args.check_only,
               "controller_or_dropout_rerun": False, "cone_policy": "visible iff cached processed observations are nonempty",
               "sensor_mount_m": [2.5, 0, .5], "sensor_yaw_rad": 0, "sensor_near_m": .1,
               "sensor_far_m": config["sensor_far_m"], "sensor_fov_rad": config["sensor_fov_rad"],
               "checked_frames": checked_count, "total_frames": len(frames), "all_frames_verified": checked_count == len(frames),
               "max_state_or_time_error": maximum_error, "visibility_transitions": transitions, "snapshots": captured}
    (output / "replay_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Verified {checked_count}/{len(frames)} frames; max error={maximum_error:.3g}; original logs unchanged.")
    print(f"Replay outputs: {output}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError, ET.ParseError) as error:
        print(f"Replay failed: {error}", file=sys.stderr)
        sys.exit(1)
