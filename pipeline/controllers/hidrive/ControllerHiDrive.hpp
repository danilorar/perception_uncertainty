/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"
#include <queue>

#define CONTROLLER_HIDRIVE_TYPE_NAME "HiDriveController"

namespace scenarioengine
{
    class ControllerHiDrive : public Controller
    {
    public:
        ControllerHiDrive(InitArgs* args);

         ~ControllerHiDrive();

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_HIDRIVE_TYPE_NAME;
        }
        virtual const char* GetTypeName() const override
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_HIDRIVE;
        }
        //virtual int GetType() const override
        //{
        //    return GetTypeStatic();
        //}
        Type GetType() const override
        {
            return CONTROLLER_TYPE_HIDRIVE;
        }

        void Init();
        //int Activate(ControlActivationMode lat_mode,
        //              ControlActivationMode long_mode,
        //              ControlActivationMode light_mode,
        //              ControlActivationMode anim_mode);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]) override;
        void ReportKeyEvent(int key, bool down);
        void SetSetSpeed(double setSpeed)
        {
            setSpeed_ = setSpeed;
        }
        void Step(double timeStep);
        /*bool hasFarTan;
        bool getHasFarTan()
        {
            return hasFarTan;
        }
        */
        void    InitPostPlayer();
        void UpdateSideLaneSensors();
        std::list<Object*> GetSensorHits();
        Object* FindLeadVehicle(bool useSensors, bool onlyFrontVehicles);
        Object*            CheckLateralPrediction();
        double CalculateADFAcceleration();
        double CalculateTurnADFAcceleration();
        double CalculateAEBAcceleration();
        double CalculateTurnAEBAcceleration();
        double             CalculateBSDAcceleration();
        void LongitudinalControl();
        double LateralControl();
        void LogValues();
        void TakeControl();
        void GetSensorInfo(double* deltaS_ego_front, double* v_front, bool* presence_front, bool use_lateral_prediction);
        const void GetKValues(double* K_d, double* K_v1, double* K_v2, double* K_p, double* K_i);
        const void GetAccelerationLimits( double* a_x_max, double* a_x_min);
        const void         GetOtherAccelerationLimits(double* calc_a_max_interp, double* calc_a_min_interp);
        void   DelayAcceleration();
        double IntersectionAcceleration();
        // void          TurnDecision(); 
        /*
        double CalculateTurnAcceleration(
                                        bool   In_turn_area,
                                        double Map_stop_line_cor,
                                        double Map_stop_line_y,
                                        double Map_stop_line_x,
                                        double a_x_ego,
                                        double a_x_turn_1,
                                        bool   turn_okay);
        */
        /* As we are not doing any lane changes, these two functions are not needed. 
        void LaneChangeCheck(double timeStep, double* trigger_LC, double* trigger_LC_abort);
        double Forced_LC_Fct();
        */
    private:
        double timeStep = 0.1;

        bool ADF_enabled     = false;  // Default value
        bool BSD_enabled     = false;
        bool baseline_enabled = false;  // This will disable all other systems!
        bool first_step_after_activation = false;  // This will be used to determine if the first step after activation has been taken.
        bool ignore_additional_vehicles = true;   // This will ignore all vehicles except the first one in the vehicles list (not counting the ego vehicle).

        bool logging_enabled = true;   // Default value
        bool lateral_prediction_enabled = ADF_enabled;
        bool syncing_enabled = true;   // Default value
        bool ISA_enabled     = false;  // was not tested
        
        double tmp_collision = 0.0;
        double           cutInDetectionTime_ = -1.0;
        vehicle::Vehicle vehicle_;
        bool             active_        = false;
        double           timeGap_       = 1.5;  // target headway time
        double           setSpeed_      = 0.0;
        double           initSpeed_     = 0.0;
        double           currentSpeed_  = 0.0;
        bool             setSpeedSet_   = false;
        bool             initSpeedSet_    = false;
        double           activationTime_     = 0.05; //delay from the start of the simulation for enabling the controller; used in  ControllerHiDrive::Step
        double           steering       = 0.0;
        double           acc            = 0.0;
        double           I_old          = 0.0;

        double           acc_pre        = 0.0;
        bool             AEB_active = false;
        double           AEB_counter        = 0.0;
        bool             AEB_warning_active = false;
        double           v_front_pre    = 0.0;
        double           a_x_front_pre      = 0.0;
        
        double           TTC_front        = 0.0;
        double           TTC_turn         = 0.0;
        double           a_required       = 0.0;  // Only used as an instance variable for logging
        
        
        double           steering_rate_ = 4.0;
        double           speedLimit  = 0.0; 
        bool             auto_lane      = true;  // This value determines whether the system should have lateral control or not.
                                                 // If set to true, the system will maintain its lateral position in the lane, if set to false it will use the lateral control from the simulink model.
        
        // ----------------- Lateral Prediction Parameters -----------------
        double lateral_diff_pre               = 0.0;
        double longitudinal_diff_pre          = 0.0;
        

        // ----------------- Acceleration Delay Parameters -----------------
        double acceleration_delay_enabled = true;
        double acceleration_delay = 0.2; //delay for reaching the assigned acceleration used for AEB and ADF; used in ControllerHiDrive::DelayAcceleration
        std::queue<double> acceleration_delay_queue;
        // ----------------- ISA Parameters -----------------
        
        double ISA_speed_reduction_mean = 5.0 / 3.6; // m/s
        double ISA_speed_reduction_std = 2.0 / 3.6; // m/s
        //unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count(); // Seed for ISA generation. Currently based on time. 

        // ----------------- ACC Parameters -----------------
//        double deceleration_max     = -3;  // No longer used. 
//        double deceleration_TTC_max = -5;  // Outdated values from the 2024-04-12 version of the HiDrive motorway code.
        double deceleration_TTC_max = -7;  // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
        double TTC_critical_AD      = 3;   // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.
        double K_PT1_ax             = 1;   // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.
        double T_ax                 = 0.2;  // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.
        double dx_min               = 5;    // m   // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.
        double timegap              = 1.6;  // s   // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.



        double vehicle_length = 4.2;    // Default value, will be updated in InitPostPlayer
        double vehicle_width = 1.8;     // Default value, will be updated in InitPostPlayer
        // ----------------- BSD Parameters -----------------
        
        double start_time     = -LARGE_NUMBER;
        double last_time_step = -LARGE_NUMBER;

        // ----------------- Side Lane Sensors -----------------
        Object* frontLeftObj       = nullptr;
        double  frontLeftObjDs   = LARGE_NUMBER;
        double  frontLeftObjDv  = 0;
        Object* frontRightObj      = nullptr;
        double  frontRightObjDs  = LARGE_NUMBER;
        double  frontRightObjDv = 0;
        Object* rearLeftObj        = nullptr;
        double  rearLeftObjDs    = LARGE_NUMBER;
        double  rearLeftObjDv   = 0;
        Object* rearRightObj       = nullptr;
        double  rearRightObjDs   = LARGE_NUMBER;
        double  rearRightObjDv  = 0;
        Object* frontFrontObj      = nullptr;
        double  frontFrontObjDs  = LARGE_NUMBER;
        double  frontFrontObjDv = 0;


        // ----------------- Lateral Control -----------------
        double u_regler_1   = 0.0;
        double y_regler     = 0.0;
        double u_regler_2   = 0.0;
        double steeringWheelAngle_ego = 0.0;
        double t_ego                  = 0;
        double t_ego_previous = 0;
        double t_ego_previous_previous = 0;

        // ----------------- Lane Change Check -----------------
        double time_since_LC = 0.0;

        // ----------------- Forced LC Function -----------------
        double forced_lane_change = 0.0;
        double max_force_lane_change = 0.0;

        // ----------------- Brake for pedestrians/bicycles -----------------
        double collisionS = -1;
        double collisionTime = -1;
        double timeEnter     = -1;
        double timeExit      = -1;
        bool   turn_AEB_active         = false;
        double turn_AEB_counter        = 0.0;
        bool   turn_AEB_warning_active = false;
        double turn_I_old          = 0.0;

    };

    Controller* InstantiateControllerHiDrive(void* args);
}  // namespace scenarioengine