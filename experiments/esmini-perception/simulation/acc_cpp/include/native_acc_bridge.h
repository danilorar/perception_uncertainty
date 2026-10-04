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
typedef struct {
    int id, category;
    double x,y,z,h,p,r,speed,length,width,height,cx,cy,cz;
} ACCSIM_Object;
typedef struct { double time_gap_s,set_speed_mps,max_acceleration,max_deceleration,lateral_distance_m; } ACCSIM_AccConfig;
typedef struct { double desired_speed_mps, observed_gap_m; int lead_id; } ACCSIM_AccOutput;
ACCSIM_API int ACCSIM_NativeAccStep(const ACCSIM_Object* ego, const ACCSIM_Object* observations,
                                   int count, const ACCSIM_AccConfig* config, double dt,
                                   ACCSIM_AccOutput* output);
#ifdef __cplusplus
}
#endif
