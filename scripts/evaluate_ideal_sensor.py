#!/usr/bin/env python3
"""Evaluate an esmini ideal object sensor and write step-by-step CSV results."""

import argparse
import csv
import ctypes as ct
import math
import time
from pathlib import Path



# DIRECTORY STRUCTURE
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIO = PROJECT_ROOT / "OSC-NCAP-scenarios" / "OpenSCENARIO" / "NCAP" / "AEB_C2C_2023" / "NCAP_AEB_C2C_CCFhol_2023.xosc"
OUTPUT = PROJECT_ROOT / "results" / "ideal_sensor.csv"
MODEL_PATH = PROJECT_ROOT / "esmini" / "resources" / "models"

# SENSOR CONFIGURATION
EGO_ID = 0 
CONTROLLER = "scenario-defined"
SENSOR_X = 4.0
SENSOR_Y = 0.0
SENSOR_Z = 0.5
SENSOR_HEADING = 0.0
NEAR_RANGE = 1.0
FAR_RANGE = 80.0
FOV_DEG = 70.0
MAX_DETECTIONS = 100
DT = 0.05
DURATION = None # set a number to impose a time limit.

# CONTROLLER CONFIGURATION
ENABLE_AEB = True
AEB_TTC_THRESHOLD = 2.0
AEB_BRAKE_DECELERATION = 8.0
AEB_LANE_HALF_WIDTH = 4.0


# VISUALIZATION PARAMS
REALTIME_FACTOR = 1.0 # 1.0 = real time, 2.0 = double speed, 0 = as fast as possible.
USE_VIEWER = True
VIEW_SENSOR_FRUSTUMS = True # NOTE: Frustum = geometric representation of the sensor's (FOV) in 3D space
VIEWER_THREADS = 1 
KEEP_VIEWER_OPEN = True


# START


class ScenarioObjectState(ct.Structure):
    
    # Fields in this class must match the fields in the C struct exactly, including their types and order
    _fields_ = [
        ("id", ct.c_int),
        ("model_id", ct.c_int),
        ("controller_type", ct.c_int),
        ("timestamp", ct.c_double),
        ("x", ct.c_double),
        ("y", ct.c_double),
        ("z", ct.c_double),
        ("h", ct.c_double),
        ("p", ct.c_double),
        ("r", ct.c_double),
        ("road_id", ct.c_uint32),
        ("junction_id", ct.c_uint32),
        ("t", ct.c_double),
        ("lane_id", ct.c_int),
        ("lane_offset", ct.c_double),
        ("s", ct.c_double),
        ("speed", ct.c_double),
        ("center_offset_x", ct.c_double),
        ("center_offset_y", ct.c_double),
        ("center_offset_z", ct.c_double),
        ("width", ct.c_double),
        ("length", ct.c_double),
        ("height", ct.c_double),
        ("object_type", ct.c_int),
        ("object_category", ct.c_int),
        ("wheel_angle", ct.c_double),
        ("wheel_rot", ct.c_double),
        ("visibility_mask", ct.c_int),
    ]

# Mapping object for readability
OBJECT_TYPES = {
    0: "none",
    1: "vehicle",
    2: "pedestrian",
    3: "misc_object",
}


def load_library(path):
    
    # Function pointers to the C functions defined in the esmini library, which are used to initialize the simulation, close it, step through time, and retrieve object states and sensor data
    library = ct.CDLL(str(path))
    library.SE_SetOption.argtypes = [ct.c_char_p] 
    library.SE_SetOption.restype = ct.c_int
    
    library.SE_SetOptionValue.argtypes = [ct.c_char_p, ct.c_char_p] # config simulation
    library.SE_SetOptionValue.restype = ct.c_int
    
    # For visualization
    library.SE_ViewerShowFeature.argtypes = [ct.c_int, ct.c_bool]
    library.SE_ViewerShowFeature.restype = None
    
    # Initialize the simulation
    library.SE_Init.argtypes = [ct.c_char_p, ct.c_int, ct.c_int, ct.c_int, ct.c_int]
    library.SE_Init.restype = ct.c_int
    
    library.SE_StepDT.argtypes = [ct.c_double]
    library.SE_StepDT.restype = ct.c_int
    
    library.SE_GetSimulationTime.restype = ct.c_double
    
    # To close the simulation
    library.SE_GetQuitFlag.restype = ct.c_int
    library.SE_Close.argtypes = []
    
    # GT nº of objects in the scenario (not from sensor)
    library.SE_GetNumberOfObjects.restype = ct.c_int
    
    # Get the state of an object (pos, v, theta) 
    library.SE_GetObjectState.argtypes = [ct.c_int, ct.POINTER(ScenarioObjectState)]
    library.SE_GetObjectState.restype = ct.c_int
    
    # For AEB, set the speed of the ego object (or any object) to simulate braking or acceleration
    library.SE_ReportObjectSpeed.argtypes = [ct.c_int, ct.c_double]
    library.SE_ReportObjectSpeed.restype = ct.c_int
    
    # Add ideal sensor to an object in the scene
    library.SE_AddObjectSensor.argtypes = [
        ct.c_int,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_int,
    ]
    library.SE_AddObjectSensor.restype = ct.c_int # if sucess returns the sensor ID else a negative value if there was an error
    
    
    # Get/Read the list of detected objects by the sensor
    library.SE_FetchSensorObjectList.argtypes = [ct.c_int, ct.POINTER(ct.c_int)]
    library.SE_FetchSensorObjectList.restype = ct.c_int # return the nº detect objects by the sensor if sucess
    return library


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi # angle in rad to be [-pi, pi]


def sensor_relative_position(ego, target, sensor_x, sensor_y, sensor_z, sensor_h):
    cos_h = math.cos(ego.h)
    sin_h = math.sin(ego.h)
    
    # Compute sensor global position based on ego position and orientation
    sensor_global_x = ego.x + cos_h * sensor_x - sin_h * sensor_y
    sensor_global_y = ego.y + sin_h * sensor_x + cos_h * sensor_y
    
    # Relative position of target w.r.t. sensor global position
    dx = target.x - sensor_global_x
    dy = target.y - sensor_global_y
    sensor_heading = ego.h + sensor_h
    
    cos_sensor = math.cos(sensor_heading)
    sin_sensor = math.sin(sensor_heading)
    
    # Relative position of targets in sensor's local coordinate frame
    local_x = cos_sensor * dx + sin_sensor * dy
    local_y = -sin_sensor * dx + cos_sensor * dy
    local_z = target.z - (ego.z + sensor_z)
    
    return local_x, local_y, local_z


# Function to parse command-line arguments when running the script (User config)
def parse_args():
    esmini_root = PROJECT_ROOT / "esmini"
    default_library = esmini_root / "bin" / "libesminiLib.so"
    built_library = esmini_root / "build" / "EnvironmentSimulator" / "Libraries" / "esminiLib" / "libesminiLib.so"
    if not default_library.exists() and built_library.exists():
        default_library = built_library
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, default=SCENARIO)
    parser.add_argument("--ego-id", type=int, default=EGO_ID)
    parser.add_argument("--controller", default=CONTROLLER)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--library", type=Path, default=default_library)
    parser.add_argument("--sensor-x", type=float, default=SENSOR_X)
    parser.add_argument("--sensor-y", type=float, default=SENSOR_Y)
    parser.add_argument("--sensor-z", type=float, default=SENSOR_Z)
    parser.add_argument("--sensor-heading", type=float, default=SENSOR_HEADING)
    parser.add_argument("--near-range", type=float, default=NEAR_RANGE)
    parser.add_argument("--far-range", type=float, default=FAR_RANGE)
    parser.add_argument("--fov-deg", type=float, default=FOV_DEG)
    parser.add_argument("--max-detections", type=int, default=MAX_DETECTIONS)
    parser.add_argument("--dt", type=float, default=DT)
    parser.add_argument("--duration", type=float, default=DURATION)
    parser.add_argument("--realtime-factor", type=float, default=REALTIME_FACTOR)
    return parser.parse_args()


def print_table_header():
    print("step  time     object  detected  type          distance  bearing  rel_x    rel_y    speed")
    print("----  -------  ------  --------  ------------  --------  -------  -------  -------  -------")


def print_table_row(step, current_time, object_id, target, detected, distance, bearing, relative_x, relative_y):
    object_type = OBJECT_TYPES.get(target.object_type, "unknown")
    print(
        f"{step:4d}  {current_time:7.3f}  {object_id:6d}  "
        f"{('yes' if detected else 'no'):8s}  {object_type:12s}  "
        f"{distance:8.2f}  {bearing:7.2f}  {relative_x:7.2f}  "
        f"{relative_y:7.2f}  {target.speed:7.2f}",
        flush=True,
    )

def calculate_ttc(ego, target, distance):
    """
    Compute time to collision based on relative speed and distance between ego and target objects. 
    If the closing speed is zero or negative, return infinity (no collision).
    """
    closing_speed = ego.speed - target.speed * math.cos(target.h - ego.h)
    if closing_speed <= 0.0:
        return math.inf
    return distance / closing_speed


def choose_aeb_target(ego, states, detected_ids, sensor_values):
    """
    Choose the target for AEB (Automatic Emergency Braking) based on the detected objects.
    The function iterates through the detected objects, calculates their relative positions and distances,
    and computes the time to collision (TTC) for each. 
    It returns the object with the minimum TTC that meets the AEB criteria 
    """
    
    candidates = []
    for object_id in detected_ids:
        target = states.get(object_id)
        if target is None or target.object_type != 1:
            continue
        relative_x, relative_y, relative_z = sensor_values[object_id]
        if relative_x <= 0.0 or abs(relative_y) > AEB_LANE_HALF_WIDTH:
            continue
        distance = math.hypot(relative_x, relative_y)
        ttc = calculate_ttc(ego, target, distance)
        candidates.append((ttc, distance, object_id))
    return min(candidates, default=(math.inf, math.inf, None))


def main():
    
    
     # To define parameters
    args = parse_args()
    
    # Load esmini library and initialize scenario
    library = load_library(args.library)
    scenario = str(args.scenario.resolve()).encode() 
    args.output.parent.mkdir(parents=True, exist_ok=True)

    if library.SE_SetOption(b"disable_stdout") != 0:
        raise RuntimeError("Could not disable esmini console logging")
    if library.SE_SetOptionValue(b"path", str(MODEL_PATH.resolve()).encode()) != 0:
        raise RuntimeError(f"Could not set esmini model path: {MODEL_PATH}")
    viewer_mode = 1 if USE_VIEWER else 0
    if library.SE_Init(scenario, 0, viewer_mode, VIEWER_THREADS, 0) != 0:
        raise RuntimeError(f"Could not initialize scenario: {args.scenario}")

    # Add an ideal sensor to ego object with specific parameters: (position, orientation, range, FOV, max detections)
    sensor_id = library.SE_AddObjectSensor(
        
        # These are defined in the command line arguments
        args.ego_id, 
        args.sensor_x,
        args.sensor_y,
        args.sensor_z,
        args.sensor_heading,
        args.near_range,
        args.far_range,
        math.radians(args.fov_deg),
        args.max_detections,
    )
    if sensor_id < 0:
        library.SE_Close()
        raise RuntimeError(f"Could not attach sensor to object {args.ego_id}")
    
    # Enable viewer and show sensor frustums if specified
    if USE_VIEWER and VIEW_SENSOR_FRUSTUMS:
        library.SE_ViewerShowFeature(1, True)

    columns = [
        "time", "controller", "ego_id", "object_id", "object_type",
        "object_category", "detected", "distance", "bearing_deg",
        "relative_x", "relative_y", "relative_z", "object_speed",
        "object_heading_deg", "lane_id", "sensor_near_range",
        "sensor_far_range", "sensor_fov_deg", "controller_state",
        "aeb_target_id", "ttc", "ego_speed", "commanded_speed",
    ]
    

    try:
        with args.output.open("w", newline="") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=columns)
            writer.writeheader()
            print_table_header()
            step = 0
            
            # Run the simulation loop until the specified duration is reached, fetching object states and sensor detections at each time step
            while (
                library.SE_GetQuitFlag() == 0
                and (args.duration is None or library.SE_GetSimulationTime() <= args.duration)
            ):
                
                # Get number of objects in the scenario (GT, not from sensor)
                number_of_objects = library.SE_GetNumberOfObjects() 
                states = {} # storage
                
                # State of each object in the scenario (pos, v, heading, etc.) -> dict {object_id: ScenarioObjectState} 
                for object_id in range(number_of_objects):
                    state = ScenarioObjectState() 
                    if library.SE_GetObjectState(object_id, ct.byref(state)) == 0: # 0 if success
                        states[object_id] = state

                # Ego obj state from states dict {object_id: ScenarioObjectState}
                ego = states.get(args.ego_id)
                if ego is None:
                    raise RuntimeError(f"Could not read ego object {args.ego_id}")

                detection_buffer = (ct.c_int * args.max_detections)()
                
                # Count nº of detected objects by the SENSOR and get their IDs
                detection_count = library.SE_FetchSensorObjectList(sensor_id, detection_buffer) 
                detected_ids = set(detection_buffer[:max(0, detection_count)])
                current_time = library.SE_GetSimulationTime()
                sensor_values = {}
                for object_id, target in states.items():
                    if object_id != args.ego_id:
                        sensor_values[object_id] = sensor_relative_position(
                            ego, target, args.sensor_x, args.sensor_y, args.sensor_z, args.sensor_heading
                        )

                
                # CONTROLLER LOGIC: AEB
                # Choose AEB target based on detected objects and their time to collide
                ttc, target_distance, aeb_target_id = choose_aeb_target(
                    ego, states, detected_ids, sensor_values
                )
                controller_state = "MONITORING"
                commanded_speed = ego.speed
                if ENABLE_AEB and ttc <= AEB_TTC_THRESHOLD:
                    controller_state = "BRAKING"
                    commanded_speed = max(0.0, ego.speed - AEB_BRAKE_DECELERATION * args.dt)
                    if library.SE_ReportObjectSpeed(args.ego_id, commanded_speed) != 0:
                        raise RuntimeError("Could not apply AEB speed command")
                if ENABLE_AEB:
                    print(
                        f"AEB: {controller_state:10s} target={aeb_target_id} "
                        f"ttc={'inf' if math.isinf(ttc) else f'{ttc:.2f}'} "
                        f"ego_speed={ego.speed:.2f} command={commanded_speed:.2f}",
                        flush=True,
                    )

                for object_id, target in states.items():
                    if object_id == args.ego_id: # skip if id = ego object
                        continue
                    
                    # Target object position w.r.t to sensor (reference frame of the sensor)
                    relative_x, relative_y, relative_z = sensor_values[object_id]
                    distance = math.sqrt(relative_x ** 2 + relative_y ** 2)
                    bearing = math.degrees(math.atan2(relative_y, relative_x))

                    print_table_row(
                        step,
                        current_time,
                        object_id,
                        target,
                        object_id in detected_ids,
                        distance,
                        bearing,
                        relative_x,
                        relative_y,
                    )
                    
                    # Save as CSV
                    writer.writerow({
                        "time": f"{current_time:.6f}",
                        "controller": args.controller,
                        "ego_id": args.ego_id,
                        "object_id": object_id,
                        "object_type": OBJECT_TYPES.get(target.object_type, "unknown"),
                        "object_category": target.object_category,
                        "detected": int(object_id in detected_ids),
                        "distance": f"{distance:.6f}",
                        "bearing_deg": f"{bearing:.6f}",
                        "relative_x": f"{relative_x:.6f}",
                        "relative_y": f"{relative_y:.6f}",
                        "relative_z": f"{relative_z:.6f}",
                        "object_speed": f"{target.speed:.6f}",
                        "object_heading_deg": f"{math.degrees(normalize_angle(target.h - ego.h)):.6f}",
                        "lane_id": target.lane_id,
                        "sensor_near_range": args.near_range,
                        "sensor_far_range": args.far_range,
                        "sensor_fov_deg": args.fov_deg,
                        "controller_state": controller_state,
                        "aeb_target_id": "" if aeb_target_id is None else aeb_target_id,
                        "ttc": "" if math.isinf(ttc) else f"{ttc:.6f}",
                        "ego_speed": f"{ego.speed:.6f}",
                        "commanded_speed": f"{commanded_speed:.6f}",
                    })

                if library.SE_StepDT(args.dt) != 0:
                    break
                step += 1
                if args.realtime_factor > 0:
                    time.sleep(args.dt / args.realtime_factor)

            if USE_VIEWER and KEEP_VIEWER_OPEN:
                input("Simulation complete. Press Enter to close the esmini viewer... ")
    finally:
        library.SE_Close()

    print(f"Wrote evaluation results to {args.output.resolve()}")


# END

if __name__ == "__main__":
    main()


