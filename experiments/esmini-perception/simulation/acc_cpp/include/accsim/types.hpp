// Author: Zhuo Ma
// Value-only interfaces. SI units; world coordinates; angles in radians.
#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace accsim {
constexpr double pi = 3.14159265358979323846;
constexpr double infinity = std::numeric_limits<double>::infinity();
struct Pose { double x=0, y=0, z=0, h=0, p=0, r=0; };
struct Box { double length=0, width=0, height=0, cx=0, cy=0, cz=0; };
struct Vector3 { double x=0,y=0,z=0; };
struct ObjectState {
    int id=-1, object_type=0, object_category=0;
    Pose pose;
    Box box;
    double speed=0;
    Vector3 velocity,acceleration;
};
struct WorldTruthFrame { double time_s=0; ObjectState ego; std::vector<ObjectState> targets; };
// This separate type is the only external-object state accepted by controllers.
struct Observation {
    int track_id=-1, object_type=0, object_category=0;
    Pose pose;
    Box box;
    double speed=0;
    Vector3 velocity,acceleration;
};
struct PerceptionFrame {
    std::uint64_t sequence=0;
    double measurement_time_s=0, delivery_time_s=0;
    std::vector<Observation> objects;
};
struct DropoutResult { PerceptionFrame frame; std::vector<int> dropped_ids; };
struct SensorConfig {
    double x_m=2.5, y_m=0, z_m=0.5, yaw_rad=0;
    double near_m=0.1, far_m=60, fov_rad=pi/3;
    int capacity=64;
};
struct AccConfig { double time_gap_s=1.5, set_speed_mps=0, lateral_distance_m=5, max_acceleration=10, max_deceleration=10; };
// Defaults come from esmini v3.8.1 ReferenceDriver::AEB, not ACC timeGap.
struct AebConfig { double ttc_s=1.5,max_deceleration_mps2=.85*9.81; };
struct ControllerInput {
    double time_s=0, dt_s=0.01;
    ObjectState ego;
    PerceptionFrame perception;
};
struct ControlRequest {
    double acceleration_mps2=0, desired_speed_mps=0;
    int lead_id=-1;
    double observed_gap_m=infinity;
    bool close_gap_stop=false;
    bool aeb_active=false;
    double observed_ttc_s=infinity;
};
struct AppliedCommand { double acceleration_mps2=0; bool limited=false; };
struct EgoMotion { ObjectState state; double progress_m=0; };
struct Geometry {
    double gap_m=infinity, closing_speed_mps=0, lateral_m=0, longitudinal_m=0;
    double distance_m=infinity;
    bool in_path=false, collision=false;
};
struct StepMetrics {
    bool collision=false;
    double minimum_distance_m=infinity, gap_m=infinity, ttc_s=infinity;
    int target_id=-1;
};
struct RunSummary {
    bool collision=false, close_gap_stop_requested=false;
    double first_collision_s=-1, collision_ego_speed_mps=-1, collision_relative_speed_mps=-1;
    double minimum_distance_m=infinity, minimum_gap_m=infinity, minimum_ttc_s=infinity;
    double first_deceleration_s=-1, minimum_acceleration_mps2=0;
    double first_aeb_s=-1;
    std::uint64_t sensor_frames=0, raw_detections=0, dropped_detections=0;
    std::uint64_t longest_missing_frames=0;
};
} // namespace accsim
