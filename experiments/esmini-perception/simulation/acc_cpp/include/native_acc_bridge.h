// Author: Zhuo Ma
// C ABI between the experiment and the library containing esmini's ORIGINAL ACC.
#pragma once
#if defined(_WIN32)
# if defined(ACCSIM_BRIDGE_BUILD)
#  define ACCSIM_API __declspec(dllexport)
# else
#  define ACCSIM_API __declspec(dllimport)
# endif
#else
# define ACCSIM_API __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif
// Nonzero when this library was compiled with graphics support.
ACCSIM_API int ACCSIM_HasViewer(void);
// Configure before SE_Init. Disabled preserves the all-target baseline.
typedef struct {
    int enabled;
    double x_m, y_m, z_m, heading_rad;
    double near_m, far_m, horizontal_fov_rad;
} ACCSIM_SensorConfig;
ACCSIM_API int ACCSIM_ConfigureSensor(const ACCSIM_SensorConfig* config);
// Copy the last controller-time detected IDs; returns count, or -1 for insufficient capacity.
ACCSIM_API int ACCSIM_GetDetectedTargetIds(int* ids, int capacity);
// Register before SE_Init; esmini owns each persistent controller instance.
ACCSIM_API void ACCSIM_RegisterPersistentAcc(void);
typedef struct {
    double time_s, input_speed_mps, desired_speed_mps, observed_gap_m;
    int lead_id, target_count;
} ACCSIM_PersistentAccOutput;
ACCSIM_API int ACCSIM_GetPersistentAccOutput(ACCSIM_PersistentAccOutput* output);
#ifdef __cplusplus
}
#endif
