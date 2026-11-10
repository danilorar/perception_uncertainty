
#include "ControllerHiDrive.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioEngine.hpp"
//#include "ScenarioGateway.hpp"
#include "ScenarioEngine.hpp"
#include "logger.hpp"
#include "playerbase.hpp"
#include <algorithm>
#include <math.h>
#include <random>
#include "fmt/printf.h"
#define LOG(...) LOG_INFO("{}", fmt::sprintf(__VA_ARGS__))

using namespace scenarioengine;
using namespace std;

/* 
This file implements the ControllerHiDrive class, which can model both an ADF and an AEB.
The code is currently very messy and if it needs to be extended in for a future project, then I would recommend it to be refactored first. 

The model has three different modes, AEB, ADF and Baseline. 
For run baseline, set the baseline_enabled variable to true in the hpp file. 
For AEB, set ADF_enabled to false and baseline_enabled to false and BSD_enabled = false.
For ADF, set ADF_enabled to true and baseline_enabled to false BSD_enabled = false.
For AEB+BSD, set  ADF_enabled to false and baseline_enabled to false and BSD_enabled = true.
*/


Controller* scenarioengine::InstantiateControllerHiDrive(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerHiDrive(initArgs);
}

ControllerHiDrive::ControllerHiDrive(InitArgs* args) : Controller(args)
{
    if (args && args->properties && args->properties->ValueExists("ADF_enabled"))
    {
        // ADF_enabled = args->properties->GetValueStr("ADF_enabled") == "true" ? true : false;
        
    }
    if (args && args->properties && args->properties->ValueExists("Speed_limit"))
    {
        speedLimit    = strtod(args->properties->GetValueStr("Speed_limit"))/3.6;  // Convert from km/h to m/s.
    }
    if (args && args->properties && args->properties->ValueExists("Enable_ISA"))
    {
        if (speedLimit != 0.0)
        {
            ISA_enabled = args->properties->GetValueStr("Enable_ISA") == "true" ? true : false;
        }
        else
        {
            LOG("Speed limit is not set. ISA cannot be enabled.");
        }
    }
    if (args && args->properties && args->properties->ValueExists("setSpeed"))
    {
        setSpeed_     = strtod(args->properties->GetValueStr("setSpeed"));
        setSpeedSet_  = true;
    }
    if (args && args->properties && args->properties->ValueExists("initSpeed"))
    {
        initSpeed_     = strtod(args->properties->GetValueStr("initSpeed"));
        currentSpeed_ = initSpeed_;
        initSpeedSet_  = true;
    }


    if (args && args->properties && args->properties->ValueExists("activationTime"))
    {
        activationTime_     = strtod(args->properties->GetValueStr("activationTime"));
    }
    if (args && args->properties && args->properties->ValueExists("EnableLogging"))
    {
        logging_enabled = args->properties->GetValueStr("EnableLogging") == "true" ? true : false;
    }

    // Testing intersection
    if (args && args->properties && args->properties->ValueExists("collisionS"))
    {
        collisionS = strtod(args->properties->GetValueStr("collisionS"));
        collisionTime = strtod(args->properties->GetValueStr("collisionTime"));
        timeEnter = strtod(args->properties->GetValueStr("timeEnter"));
        timeExit = strtod(args->properties->GetValueStr("timeExit"));
        if (timeEnter > timeExit)
        {
            double tmp_collision = timeEnter;
			timeEnter = timeExit;
			timeExit = tmp_collision;
        }
    }

}

ControllerHiDrive::~ControllerHiDrive()
{
    // Cleanup code goes here.
    // If you allocated any resources in the constructor or other member functions, free them here.

    // Example: If you had a dynamically allocated pointer, you'd delete it like this:
    // delete somePointer;

    // If base class `Controller` has any virtual destructor, it will be called automatically.
}

void ControllerHiDrive::Init()
{
    Controller::Init();
}

void ControllerHiDrive::InitPostPlayer()
{
    vehicle_length = object_->boundingbox_.dimensions_.length_;
    vehicle_width = object_->boundingbox_.dimensions_.width_;
    
    player_->AddObjectSensor(object_, 2.0f, 0.0f, 1.5f, 0.0f, 1.0f, 200.0f, M_PI / 3, 1000);                         // 0, LC1  LiDAR + Camera
    player_->AddObjectSensor(object_, 4.0f, 0.0f, 0.5f, 0.0f, 1.0f, 250.0f, M_PI / 6, 1000);                         // 1, R1   Long range Radar
    
    if (ADF_enabled or BSD_enabled)
    {
        player_->AddObjectSensor(object_, 1.5f, 0.0f, 0.5f, 0.0f, 1.0f, 30.0f, 2.0f * M_PI, 1000);                       // 2, C2   Peripheral Cameras
        player_->AddObjectSensor(object_, -1.0f, -1.0f, 0.5f, 190.0f * M_PI / 180.0f, 1.0f, 250.0f, M_PI / 6.0f, 1000);  // 3, R2   Rear Right
        player_->AddObjectSensor(object_, -1.0f, 1.0f, 0.5f, 170.0f * M_PI / 180.0f, 1.0f, 250.0f, M_PI / 6.0f, 1000);   // 4, R3   Rear Left
        player_->AddObjectSensor(object_, 4.0f, 1.0f, 0.5f, M_PI / 3, 1.0f, 150.0f, 2 * M_PI / 3, 1000);                 // 5, R4   Front Left
        player_->AddObjectSensor(object_, 4.0f, -1.0f, 0.5f, -M_PI / 3, 1.0f, 150.0f, 2 * M_PI / 3, 1000);               // 6, R5   Front Right
        player_->AddObjectSensor(object_, -1.0f, 0.0f, 0.5f, M_PI, 1.0f, 100.0f, M_PI / 2, 1000);  // 7, R6   Short range Radar - Rear Center
	}
    
}

//int ControllerHiDrive::Activate(ControlActivationMode lat_mode,
//                                 ControlActivationMode long_mode,
//                                 ControlActivationMode light_mode,
//                                 ControlActivationMode anim_mode)
int ControllerHiDrive::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    ControlActivationMode lat_mode   = mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)];
    ControlActivationMode long_mode  = mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)];
    ControlActivationMode light_mode = mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LIGHT)];
    ControlActivationMode anim_mode  = mode[static_cast<unsigned int>(ControlDomains::DOMAIN_ANIM)];

    currentSpeed_ = object_->GetSpeed();

    if (object_)
    {
        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
        vehicle_.SetMaxDec(object_->GetMaxDeceleration());
        vehicle_.SetSteeringRate(steering_rate_);
    }
    //Controller::Activate(lat_mode,
    //                    long_mode,
    //                    light_mode,
    //                    anim_mode);
    Controller::Activate(mode);

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
    // Make sure heading is aligned with road driving direction
        object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    }
    if (player_)
    {
        //player_->SteeringSensorSetVisible(object_->GetId(), true);
        player_->LookaheadSensorSetVisible(object_->GetId(), true);
    }
    return 0;
}

void ControllerHiDrive::ReportKeyEvent(int key, bool down)
{
        (void)key;
        (void)down;
}

void ControllerHiDrive::DelayAcceleration()
{
    acceleration_delay_queue.push(acc);
    if (scenario_engine_->getSimulationTime() >= acceleration_delay && !acceleration_delay_queue.empty())
    {
        acc = acceleration_delay_queue.front();
		acceleration_delay_queue.pop();
	}
	else
	{
		acc = 0;
	}
}

void ControllerHiDrive::Step(double timeStep)
{
    if (object_->GetName() == "object_2" or object_->GetName() == "vehicle_2")
    {
        return;
    }
    this->timeStep = timeStep;
    if (scenario_engine_->getSimulationTime() >= activationTime_ && mode_ == ControlOperationMode::MODE_ADDITIVE && ADF_enabled && !syncing_enabled)
    {
        this->TakeControl();
    }
    if (mode_ == ControlOperationMode::MODE_ADDITIVE)
    {
        currentSpeed_ = object_->GetSpeed();
    }

    LongitudinalControl();
    if (acceleration_delay_enabled)
    {
        DelayAcceleration();
    }
    if (mode_ == ControlOperationMode::MODE_OVERRIDE)
    {
        if (!first_step_after_activation)
        {
			first_step_after_activation = true;
		}
        else
        {
            currentSpeed_ += timeStep * acc;
            if (currentSpeed_ < 0)
            {
                currentSpeed_ = 0;
            }
            //gateway_->updateObjectSpeed(object_->GetId(), 0.0, currentSpeed_);
            object_->SetSpeed(currentSpeed_);
            if (auto_lane || ADF_enabled) // Not sure exactly why but if ADF is disabled, we must use the MoveAlongS function to move the vehicle.
            {
                object_->MoveAlongS(currentSpeed_ * timeStep);
                //gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
            }
            else
            {
                vehicle_.SetThrottleDisabled(
                    true);  // This disables the throttle part of the DrivingControlAnalog, which is useful as we deal with acceleration seperately.
                vehicle_.speed_ = currentSpeed_;
                steering        = LateralControl();

                vehicle_.wheelAngle_ = steering;
                //vehicle_.Update(timeStep);
                //gateway_->updateObjectWorldPosXYH(object_->GetId(), 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
                //gateway_->updateObjectWheelAngle(object_->GetId(), 0.0, vehicle_.wheelAngle_);
                vehicle_.Update(timeStep);
                object_->pos_.SetInertiaPos(vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
                object_->dirty_.SetBits(Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL);
                object_->wheel_angle_ = vehicle_.wheelAngle_;
                object_->dirty_.SetBits(Object::DirtyBit::WHEEL_ANGLE);
            }
        }
    }
    if (logging_enabled)
    {
        LogValues();
    }
    Controller::Step(timeStep);
}

void ControllerHiDrive::TakeControl()
{
    // This function is called when the controller should take control of the vehicle.
    // Before this function is called, the controller should not change the speed or steering of the vehicle.
    // Note that the controller must start in additive mode to work correctly. 
    if (baseline_enabled or mode_ == ControlOperationMode::MODE_OVERRIDE)
    {  // We are already in control. Do nothing.
        return;
    }
    if (not setSpeedSet_)
    {
        setSpeed_     = object_->GetSpeed();
        setSpeedSet_  = true;
        currentSpeed_ = setSpeed_;
        LOG("Set speed and initial speed was set to %f m/s", setSpeed_);
    }
    if (initSpeedSet_ && ADF_enabled)   // I don't think it makes sense to adjust speed if ADF is not enabled.
    {
        currentSpeed_ = initSpeed_;
        LOG("Initial speed was set to %f m/s", setSpeed_);
    }
    if (ISA_enabled)
    {
        if (currentSpeed_ > speedLimit*1.03)
        {
            //std::default_random_engine       generator(seed);
            std::default_random_engine       generator(12);
            std::normal_distribution<double> distribution(ISA_speed_reduction_mean, ISA_speed_reduction_std);   // Divide by 3.6 to convert from km/h to m/s.
            double speedLimitRandom = distribution(generator);
            if (speedLimitRandom > 0)
            {
                currentSpeed_ = max(currentSpeed_ - speedLimitRandom, speedLimit);
                LOG("Speed was adjusted to %f m/s by ISA", currentSpeed_);
			}
            
		}
	}
    cutInDetectionTime_ = scenario_engine_->getSimulationTime();
    mode_ = ControlOperationMode::MODE_OVERRIDE;
}

list<Object*> ControllerHiDrive::GetSensorHits()
{
    // This function returns a list of objects that are detected by the sensors of the vehicle.
    // Objects detected by multiple sensors are only included once in the list.
    // This object will not be included in the list. 

    list<Object*> hits;
    for (ObjectSensor* sensor : player_->sensor)
    {
        for (int i = 0; i < sensor->nObj_; i++)
        {
            Object* obj           = sensor->hitList_[i].obj_;
            if (obj == object_)
            {
                continue;
            }
            if (ignore_additional_vehicles && obj != (entities_->object_[0] != object_ ? entities_->object_[0] : entities_->object_[1]))
            {
				continue;
			}
            
            bool    alreadyInList = false;
            for (Object* hit : hits)
            {
                if (obj == hit)
                {
                    alreadyInList = true;
                    break;
                }
            }
            if (!alreadyInList)
            {
                hits.push_back(obj);
            }
        }
    }
    return hits;
}

Object* ControllerHiDrive::FindLeadVehicle(bool useSensors, bool onlyFrontVehicles = true)
{
    // This function returns the object that is closest to this vehicle, in the same lane and in front of it.
    double minGapLength = LARGE_NUMBER;

    list<Object*> hits;
    if (useSensors && scenario_engine_->getSimulationTime() >= activationTime_)
    {
        hits = GetSensorHits();
    }
    else
    {
        hits = list<Object*>();
        if (ignore_additional_vehicles)
        {
            hits.push_back(entities_->object_[0] != object_ ? entities_->object_[0] : entities_->object_[1]);
        }
        else
        {
            for (Object* obj : entities_->object_)
            {
                if (obj == nullptr || obj == object_)
                {
                    continue;
                }
                hits.push_back(obj);
            }
        }
        
    }

    Object* leadObj = nullptr;
    for (Object* hit : hits)
    {
        roadmanager::PositionDiff diff;
        object_->pos_.Delta(&hit->pos_, diff, false);
        double x_local, y_local;
        double dist = object_->FreeSpaceDistance(hit, &y_local, &x_local);

        if ((!onlyFrontVehicles || diff.dLaneId == 0 && diff.ds > 0) && dist < minGapLength)
        {
            leadObj      = hit;
            minGapLength = dist;
        }
    }
    return leadObj;
}

Object* ControllerHiDrive::CheckLateralPrediction()
{
    list<Object*> hits = GetSensorHits();
    Object*       leadObj = nullptr;
    double        x_local, y_local;
    double min_ds = LARGE_NUMBER;
    for (Object* hit : hits)
    {
        roadmanager::PositionDiff diff;
        object_->pos_.Delta(&hit->pos_, diff, false);

        if (diff.dLaneId != 0 && diff.ds > 0 && diff.ds < min_ds)
        {
            leadObj      = hit;
            min_ds = diff.ds;
        }
    }
    if (leadObj)
    {
        LOG("Lead obj name: %s", leadObj->GetName().c_str());
        double TTC_threshold = 3;
        double dy_threshold  = 2;

        
        double dist = object_->FreeSpaceDistance(leadObj, &y_local, &x_local);
        double velocity_diff_x = (x_local - longitudinal_diff_pre) / timeStep;
        double velocity_diff_y = (y_local - lateral_diff_pre) / timeStep;

        longitudinal_diff_pre = x_local;
        lateral_diff_pre = y_local;

        //double TTC_obj_0 = x_local / (currentSpeed_ - Obj_vx_0);
        double TTC_obj_0 = x_local / (-velocity_diff_x);
        LOG("TTC_obj_0: %f", TTC_obj_0);
        LOG("x_local: %f", x_local);
        LOG("y_local: %f", y_local);
        LOG("dist: %f", dist);
        LOG("velocity_diff_x: %f", velocity_diff_x);

        if (TTC_obj_0 < 0 || TTC_obj_0 >= TTC_threshold)
        {
            LOG("TTC_obj_0 is not valid: %f", TTC_obj_0);
            return nullptr;
        }
        double dy_predict_0 = y_local + velocity_diff_y * TTC_obj_0;

        
        LOG("Longitudinal check passed. ");
        if (TTC_obj_0 < TTC_threshold && abs(dy_predict_0) < dy_threshold && x_local > 0)
        {
            return leadObj;
        }
    }
    return nullptr;

}

void ControllerHiDrive::UpdateSideLaneSensors()
{
    
    if (scenario_engine_->getSimulationTime() < activationTime_)
    {
        return;
    }
        
    list<Object*> hits = GetSensorHits();
    

    for (Object* hit : hits)
    {
        roadmanager::PositionDiff diff;
        if (not object_->pos_.Delta(&hit->pos_, diff, true))
        {
			continue;
		}
        if (diff.dLaneId == 0 and diff.ds > 0)
        {
            if (frontFrontObj == nullptr or diff.ds < frontFrontObjDs)
            {
				frontFrontObj = hit;
				frontFrontObjDs = diff.ds;
                frontFrontObjDv = currentSpeed_ - hit->GetSpeed();
			}
        }
        if (diff.dLaneId == 1 and diff.ds > 0)
        {
            if (frontLeftObj == nullptr or diff.ds < frontLeftObjDs)
            {
                frontLeftObj = hit;
				frontLeftObjDs = diff.ds;
                frontLeftObjDv = currentSpeed_ - hit->GetSpeed();
            }
        }
        if (diff.dLaneId == -1 and diff.ds > 0)
        {
            if (frontRightObj == nullptr or diff.ds < frontRightObjDs)
            {
				frontRightObj = hit;
                frontRightObjDs = diff.ds;
                frontRightObjDv = currentSpeed_ - hit->GetSpeed();
            }
        }
        if (diff.dLaneId == 1 and diff.ds < 0)
        {
		    if (rearLeftObj == nullptr or -diff.ds < rearLeftObjDs)
            {
                rearLeftObj = hit;
				rearLeftObjDs = -diff.ds;
                rearLeftObjDv = currentSpeed_ - hit->GetSpeed();
            }
        }
        if (diff.dLaneId == -1 and diff.ds < 0)
        {
            if (rearRightObj == nullptr or -diff.ds < rearRightObjDs)
            {
				rearRightObj = hit;
                rearRightObjDs = -diff.ds;
                rearRightObjDv = currentSpeed_ - hit->GetSpeed();
            }
        }

    }

    return;
}

void ControllerHiDrive::GetSensorInfo(double* deltaS_ego_front, double* v_front, bool* presence_front, bool use_lateral_prediction = false)
{
    Object* leadVehicle = nullptr;
    if (lateral_prediction_enabled && use_lateral_prediction)
    {
		leadVehicle = CheckLateralPrediction();
        
    }
    if (leadVehicle == nullptr)
    {
		leadVehicle = FindLeadVehicle(true);
    }
    else
    {
        LOG("Lateral prediction is used to find the lead vehicle.");
    }
    if (leadVehicle == nullptr)
    {
        *presence_front   = false;
        *v_front          = 0;
        *deltaS_ego_front = LARGE_NUMBER;
    }
    else
    {
        *presence_front = true;
        // *v_front        = leadVehicle->GetSpeed()*cos(object_->pos_.GetH()-leadVehicle->pos_.GetH());            // This only considers the lead vehicle's velocity in the ego vehicle's heading direction. 
        *v_front = leadVehicle->GetSpeed();

        double x_local, y_local;
        double dist       = object_->FreeSpaceDistance(leadVehicle, &y_local, &x_local);
        *deltaS_ego_front = dist;
    }
}

const void ControllerHiDrive::GetKValues(double* K_d, double* K_v1, double* K_v2, double* K_p, double* K_i)
{
    if (currentSpeed_ <= 60 / 3.6)
    {
        *K_d  = 1.0;        // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
		*K_v1 = 0.5;
		*K_v2 = 0.4;
		*K_p  = 0.1;
		*K_i  = 1;
    }
    else if (currentSpeed_ <= 100 / 3.6)
    {
        *K_d  = 1.0;        // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
        *K_v1 = 0.5;
        *K_v2 = 0.4;
        *K_p  = 0.1;
        *K_i  = 1;
    }
    else
    {
        *K_d  = 1.0;        // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
        *K_v1 = 0.5;
        *K_v2 = 0.4;
        *K_p  = 0.1;
        *K_i  = 1;
    }
}

const void ControllerHiDrive::GetAccelerationLimits(double* a_x_max, double* a_x_min)

{
    // This function calculates the minimum and maximum acceleration values based on the current speed.
    // If the speed is smaller than the first value in "v_values", the first value in "a_x_max_values" and "a_x_min_values" is used.
    // If the speed is larger than the last value in "v_values", the last value in "a_x_max_values" and "a_x_min_values" is used.
    // If the speed is between two values in "v_values", the corresponding values in "a_x_max_values" and "a_x_min_values" are interpolated.

    vector<double> a_x_max_values = {4, 2};     // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
    // vector<double> a_x_min_values = {-5, -3.5}; // Outdated values from the 2024-04-12 version of the HiDrive motorway code.
    vector<double> a_x_min_values = {-6, -5};   // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
    vector<double> v_values       = {5, 20};    // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.

    if (currentSpeed_ <= v_values.front())
    {
        *a_x_max = a_x_max_values.front();
        *a_x_min = a_x_min_values.front();
    }
    else if (currentSpeed_ >= v_values.back())
    {
        *a_x_max = a_x_max_values.back();
        *a_x_min = a_x_min_values.back();
    }
    else
    {
        for (int i = 0; i < v_values.size() - 1; i++)
        {
            if (currentSpeed_ >= v_values[i] && currentSpeed_ < v_values[i + 1])
            {
                *a_x_max =
                    a_x_max_values[i] + (a_x_max_values[i + 1] - a_x_max_values[i]) * (currentSpeed_ - v_values[i]) / (v_values[i + 1] - v_values[i]);
                *a_x_min =
                    a_x_min_values[i] + (a_x_min_values[i + 1] - a_x_min_values[i]) * (currentSpeed_ - v_values[i]) / (v_values[i + 1] - v_values[i]);
                break;
            }
        }
    }
}

const void ControllerHiDrive::GetOtherAccelerationLimits(double* calc_a_max_interp, double* calc_a_min_interp)
{
    const int nValues               = 6;
                                                                                                        // Remember to update the values here in the turn function as well.
    double    v_interp[nValues] = {0 / 3.6, 10 / 3.6, 80 / 3.6, 129 / 3.6, 130 / 3.6, 999 / 3.6};   // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
    double    a_max_interp[nValues] = {2.5, 2.5, 1.5, 1.5, 0.1, 0.1};       	                        // Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.
    double    a_min_interp[nValues] = {-6, -6, -5, -5, -5, -5};											// Verified as up-to-date with the 2024-06-18 version of the HiDrive motorway code.

    if (currentSpeed_ < v_interp[0])
    {
        *calc_a_max_interp = a_max_interp[0];
        *calc_a_min_interp = a_min_interp[0];
    }
    else if (currentSpeed_ > v_interp[nValues - 1])
    {
        *calc_a_max_interp = a_max_interp[nValues - 1];
        *calc_a_min_interp = a_min_interp[nValues - 1];
    }
    else
    {
        for (int i = 0; i < nValues - 1; i++)
        {
            if (currentSpeed_ >= v_interp[i] && currentSpeed_ < v_interp[i + 1])
            {
                *calc_a_max_interp = (a_max_interp[i] + (a_max_interp[i + 1] - a_max_interp[i]) * (currentSpeed_ - v_interp[i]) /
                                                           (v_interp[i + 1] - v_interp[i]));
                *calc_a_min_interp = (a_min_interp[i] + (a_min_interp[i + 1] - a_min_interp[i]) * (currentSpeed_ - v_interp[i]) /
                                                            (v_interp[i + 1] - v_interp[i]));
                break;
            }
        }
    }
}

double ControllerHiDrive::CalculateADFAcceleration()
{
    // Sensor Module
    bool   presence_front;
    double v_front;
    double deltaS_ego_front;
    GetSensorInfo(&deltaS_ego_front, &v_front, &presence_front, true);
    double dv_current = presence_front ? currentSpeed_ - v_front : 0;
    double Obj_x      = (presence_front ? deltaS_ego_front : 250) - vehicle_length;

    if (syncing_enabled && presence_front && mode_ == ControlOperationMode::MODE_ADDITIVE && scenario_engine_->getSimulationTime() >= activationTime_ && ADF_enabled)
    {
        TakeControl();
    }

    // ACC Parameters
    double K_d;
    double K_v1;
    double K_v2;
    double K_p;
    double K_i;
    GetKValues(&K_d, &K_v1, &K_v2, &K_p, &K_i);

    // L3Pilot_Mature_AEB
    double a_x_front;
    if (presence_front)
    {
        a_x_front = (v_front - v_front_pre) / timeStep;
    }
    else
    {
        a_x_front = 0;
    }
    if (abs(a_x_front) > 15)
    {
        a_x_front = 0;
    }
    if (a_x_front > 0)
    {
        a_x_front = 0;
    }
    

    
    double a_x_front_PT1 = T_ax * (K_PT1_ax * a_x_front - a_x_front_pre) + a_x_front_pre;

    // ACC Parameters
    double a_x_max;
    double a_x_min;
    this->GetAccelerationLimits(&a_x_max, &a_x_min);

    // ACC control

    double dv_des = K_d * (Obj_x - (timegap * currentSpeed_ + dx_min));
    if (dv_des > 10)
    {
        dv_des = 10;
    }

    // Factor applied to the acceleration depends on the time gap to the
    // predecessor(the closer the object, the stronger the acceleraiton is % considered)
    double current_THW = (Obj_x - dx_min) / currentSpeed_;
    double K_a         = fmin(1, fmax(0.2, 0.2 - 0.7 / (2.5 - 0.9) * (current_THW - 2.5)));
    double a_x_front_PT1_in;
    if (a_x_front_PT1 < -1.5)
    {
        a_x_front_PT1_in = a_x_front_PT1;
    }
    else
    {
        a_x_front_PT1_in = 0;
    }
    double dv_current_value;
    if (fabs(dv_current) > 1)
    {
        dv_current_value = dv_current;
    }
    else
    {
        dv_current_value = 0;
    }
    double a_des_d = K_v1 * (dv_des - dv_current_value) + K_a * a_x_front_PT1_in;
    double a_des_v = K_v2 * (setSpeed_ - currentSpeed_);

    double a_des   = fmin(a_des_d, a_des_v);
    double a_des_f = CLAMP(a_des, a_x_min, a_x_max);

    I_old     = (a_des_f - acc) * (K_i * timeStep) + I_old;
    double a_des_PI1 = a_des_f + (a_des_f - acc) * K_p + I_old;

    double a_des_PI = a_des_PI1;
    // TTC Calculation
    if (dv_current > 0 && Obj_x < 200)
    {
        TTC_front = Obj_x / dv_current;
    }
    else
    {
        TTC_front = 999;
    }

    // Limit Acceleration according to Hi-Drive
    
    double    calc_a_max_interp;
    double    calc_a_min_interp;
    GetOtherAccelerationLimits(&calc_a_max_interp, &calc_a_min_interp);
    
    if (a_des_PI > calc_a_max_interp)
    {
        a_des_PI = calc_a_max_interp;
    }

    
    if (TTC_front < TTC_critical_AD)
    {
        if (a_des_PI < deceleration_TTC_max)
        {
            a_des_PI = deceleration_TTC_max;
        }
    }
    else
    {
        if (a_des_PI < calc_a_min_interp)
        {
            a_des_PI = calc_a_min_interp;
        }
    }
    a_x_front_pre = a_x_front_PT1;
    v_front_pre   = v_front;
    return a_des_PI;
}

double ControllerHiDrive::CalculateTurnADFAcceleration()
{
    // This function should probably be merged with the regular ADFAcceleration function but for now it is kept separate.

    // Sensor Module
    double dv_current = currentSpeed_;
    double Obj_x      = collisionS - object_->pos_.GetS() - vehicle_length;

    if (syncing_enabled && mode_ == ControlOperationMode::MODE_ADDITIVE &&
        scenario_engine_->getSimulationTime() >= activationTime_ && ADF_enabled)
    {
        TakeControl();
    }

    // ACC Parameters
    double K_d;
    double K_v1;
    double K_v2;
    double K_p;
    double K_i;
    GetKValues(&K_d, &K_v1, &K_v2, &K_p, &K_i);

    // L3Pilot_Mature_AEB
    double a_x_front_PT1 = 0;

    // ACC Parameters
    double a_x_max;
    double a_x_min;
    this->GetAccelerationLimits(&a_x_max, &a_x_min);

    // ACC control

    double dv_des = K_d * (Obj_x - (timegap * currentSpeed_ + dx_min));
    if (dv_des > 10)
    {
        dv_des = 10;
    }

    // Factor applied to the acceleration depends on the time gap to the
    // predecessor(the closer the object, the stronger the acceleraiton is % considered)
    double current_THW = (Obj_x - dx_min) / currentSpeed_;
    double K_a         = fmin(1, fmax(0.2, 0.2 - 0.7 / (2.5 - 0.9) * (current_THW - 2.5)));
    double a_x_front_PT1_in;
    if (a_x_front_PT1 < -1.5)
    {
        a_x_front_PT1_in = a_x_front_PT1;
    }
    else
    {
        a_x_front_PT1_in = 0;
    }
    double dv_current_value;
    if (fabs(dv_current) > 1)
    {
        dv_current_value = dv_current;
    }
    else
    {
        dv_current_value = 0;
    }
    double a_des_d = K_v1 * (dv_des - dv_current_value) + K_a * a_x_front_PT1_in;
    double a_des_v = K_v2 * (setSpeed_ - currentSpeed_);

    double a_des   = fmin(a_des_d, a_des_v);
    double a_des_f = CLAMP(a_des, a_x_min, a_x_max);

    turn_I_old            = (a_des_f - acc) * (K_i * timeStep) + turn_I_old;
    double a_des_PI1 = a_des_f + (a_des_f - acc) * K_p + turn_I_old;

    double a_des_PI = a_des_PI1;
    // TTC Calculation
    if (dv_current > 0 && Obj_x < 200)
    {
        TTC_front = Obj_x / dv_current;
    }
    else
    {
        TTC_front = 999;
    }

    // Limit Acceleration according to Hi-Drive
    double calc_a_max_interp;
    double calc_a_min_interp;
    GetOtherAccelerationLimits(&calc_a_max_interp, &calc_a_min_interp);

    if (a_des_PI > calc_a_max_interp)
    {
        a_des_PI = calc_a_max_interp;
    }

    if (TTC_front < TTC_critical_AD)
    {
        if (a_des_PI < deceleration_TTC_max)
        {
            a_des_PI = deceleration_TTC_max;
        }
    }
    else
    {
        if (a_des_PI < calc_a_min_interp)
        {
            a_des_PI = calc_a_min_interp;
        }
    }
    return a_des_PI;
}

double ControllerHiDrive::CalculateAEBAcceleration()
{
    bool   presence_front;
    double v_front;
    double deltaS_ego_front;
    GetSensorInfo(&deltaS_ego_front, &v_front, &presence_front);
    double dv_current = presence_front ? currentSpeed_ - v_front : 0;
    double Obj_x      = (presence_front ? deltaS_ego_front : 250) - vehicle_length;

    // AEB parameters
    double t_brake_stage_1         = 0.4;
    double a_x_build_up_velocity_1 = 35;
    double a_x_build_up_velocity_2 = 35;

    double x_remain = 0.5;
    if (dv_current > -0.5)
    {
        a_required = std::min(acc_pre, -(pow(dv_current, 2)) / (2 * (Obj_x - x_remain)));
    }
    else
    {
        a_required = -1;
    }

    double v_AEB[17]                 = {0 / 3.6,
                                        10 / 3.6,
                                        20 / 3.6,
                                        30 / 3.6,
                                        40 / 3.6,
                                        50 / 3.6,
                                        60 / 3.6,
                                        70 / 3.6,
                                        80 / 3.6,
                                        90 / 3.6,
                                        100 / 3.6,
                                        110 / 3.6,
                                        120 / 3.6,
                                        130 / 3.6,
                                        140 / 3.6,
                                        150 / 3.6,
                                        250 / 3.6};

    double threshold_AEB_warning[17] = {1.2, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
    double threshold_AEB[17];
    if (v_front > 1)
    {  // Front vehicle Moving
        threshold_AEB[0]  = 0.85;
        threshold_AEB[1]  = 0.85;
        threshold_AEB[2]  = 0.9;
        threshold_AEB[3]  = 0.95;
        threshold_AEB[4]  = 1.1;
        threshold_AEB[5]  = 1.2;
        threshold_AEB[6]  = 1.3;
        threshold_AEB[7]  = 1.4;
        threshold_AEB[8]  = 1.5;
        threshold_AEB[9]  = 1.5;
        threshold_AEB[10] = 1.5;
        threshold_AEB[11] = 1.5;
        threshold_AEB[12] = 1.5;
        threshold_AEB[13] = 1.5;
        threshold_AEB[14] = 1.5;
        threshold_AEB[15] = 1.5;
        threshold_AEB[16] = 1.5;
    }
    else
    {  // Front vehicle stand still}
        threshold_AEB[0]  = 0.75;
        threshold_AEB[1]  = 0.75;
        threshold_AEB[2]  = 0.8;
        threshold_AEB[3]  = 0.95;
        threshold_AEB[4]  = 1.1;
        threshold_AEB[5]  = 1.2;
        threshold_AEB[6]  = 1.4;
        threshold_AEB[7]  = 1.4;
        threshold_AEB[8]  = 1.4;
        threshold_AEB[9]  = 1.4;
        threshold_AEB[10] = 1.4;
        threshold_AEB[11] = 1.4;
        threshold_AEB[12] = 1.4;
        threshold_AEB[13] = 1.4;
        threshold_AEB[14] = 1.4;
        threshold_AEB[15] = 1.4;
        threshold_AEB[16] = 1.4;
    }
    double current_threshold_warning = 0;
    double current_threshold         = 0;
    for (int i = 0; i < 16; i++)
    {
        if (currentSpeed_ >= v_AEB[i] && currentSpeed_ < v_AEB[i + 1])
        {
            current_threshold_warning = threshold_AEB_warning[i] + (threshold_AEB_warning[i + 1] - threshold_AEB_warning[i]) *
                                                                       (currentSpeed_ - v_AEB[i]) / (v_AEB[i + 1] - v_AEB[i]);
            current_threshold = threshold_AEB[i] + (threshold_AEB[i + 1] - threshold_AEB[i]) * (currentSpeed_ - v_AEB[i]) / (v_AEB[i + 1] - v_AEB[i]);
        }
    }

    // AEB Warning
    if (TTC_front < current_threshold_warning)
    {
        AEB_warning_active = 1;
    }
    else if (AEB_warning_active == 1 && a_required >= 0)
    {
        AEB_warning_active = 1;
    }
    else
    {
        AEB_warning_active = 0;
    }
    // AEB Control
    if (TTC_front < current_threshold)
    {
        AEB_active = 1;
        if (mode_ == ControlOperationMode::MODE_ADDITIVE && scenario_engine_->getSimulationTime() >= activationTime_ && !ADF_enabled)
        {
            this->TakeControl();
        }
        AEB_counter = AEB_counter + timeStep;
    }
    else
    {
        if (AEB_active == 1 && a_required > 1)
        {
            AEB_counter = AEB_counter + timeStep;
        }
        else if (ADF_enabled == true)
        {
            AEB_active  = 0;
            AEB_counter = 0;
        }
    }
    double a_AEB_brake;
    if (AEB_active == 1)
    {
        if (AEB_counter <= t_brake_stage_1)
        {
            a_AEB_brake = std::max(acc_pre - a_x_build_up_velocity_1 * timeStep, -3.5);
        }
        else
        {
            a_AEB_brake = std::max(acc_pre - a_x_build_up_velocity_2 * timeStep, std::max(a_required, -9.81));
        }
    }
    else
    {
        a_AEB_brake = LARGE_NUMBER;
    }
    return a_AEB_brake;
}

double ControllerHiDrive::CalculateTurnAEBAcceleration()
{
    // This function should probably be merged with the regular AEBAcceleration function but for now it is kept separate.


    if (scenario_engine_->getSimulationTime() > timeExit)
    {
        return LARGE_NUMBER;
	}
    double dv_current = currentSpeed_;
    double Obj_x      = collisionS - object_->pos_.GetS() - vehicle_length;

    // AEB parameters
    double t_brake_stage_1         = 0.4;
    double a_x_build_up_velocity_1 = 35;
    double a_x_build_up_velocity_2 = 35;

    double x_remain = 0.5;
    if (dv_current > -0.5)
    {
        a_required = std::min(acc_pre, -(pow(dv_current, 2)) / (2 * (Obj_x - x_remain)));
    }
    else
    {
        a_required = -1;
    }
    double v_AEB[17] = {0 / 3.6,
                        10 / 3.6,
                        20 / 3.6,
                        30 / 3.6,
                        40 / 3.6,
                        50 / 3.6,
                        60 / 3.6,
                        70 / 3.6,
                        80 / 3.6,
                        90 / 3.6,
                        100 / 3.6,
                        110 / 3.6,
                        120 / 3.6,
                        130 / 3.6,
                        140 / 3.6,
                        150 / 3.6,
                        250 / 3.6};

    double threshold_AEB_warning[17] = {1.2, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
    double threshold_AEB[17]         = {1.0, 1.0, 1.0, 1.0, 1.1, 1.2, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4};
    
    double current_threshold_warning = 0;
    double current_threshold         = 0;
    for (int i = 0; i < 16; i++)
    {
        if (currentSpeed_ >= v_AEB[i] && currentSpeed_ < v_AEB[i + 1])
        {
            current_threshold_warning = threshold_AEB_warning[i] + (threshold_AEB_warning[i + 1] - threshold_AEB_warning[i]) *
                                                                       (currentSpeed_ - v_AEB[i]) / (v_AEB[i + 1] - v_AEB[i]);
            current_threshold = threshold_AEB[i] + (threshold_AEB[i + 1] - threshold_AEB[i]) * (currentSpeed_ - v_AEB[i]) / (v_AEB[i + 1] - v_AEB[i]);
        }
    }

    // AEB Warning
    if (TTC_turn < current_threshold_warning)
    {
        turn_AEB_warning_active = 1;
    }
    else if (turn_AEB_warning_active == 1 && a_required >= 0)
    {
        turn_AEB_warning_active = 1;
    }
    else
    {
        turn_AEB_warning_active = 0;
    }
    // AEB Control
    
    if (TTC_turn < current_threshold)
    {
        turn_AEB_active = 1;
        if (mode_ == ControlOperationMode::MODE_ADDITIVE && scenario_engine_->getSimulationTime() >= activationTime_ && !ADF_enabled)
        {
            this->TakeControl();
        }
        turn_AEB_counter = turn_AEB_counter + timeStep;
    }
    else
    {
        if (turn_AEB_active == 1 && a_required > 1)
        {
            turn_AEB_counter = turn_AEB_counter + timeStep;
        }
        else if (ADF_enabled == true)
        {
            turn_AEB_active = 0;
            turn_AEB_counter = 0;
        }
    }
    double a_AEB_brake;
    if (turn_AEB_active == 1)
    {
        if (turn_AEB_counter <= t_brake_stage_1)
        {
            a_AEB_brake = std::max(acc_pre - a_x_build_up_velocity_1 * timeStep, -3.5);
        }
        else
        {
            a_AEB_brake = std::max(acc_pre - a_x_build_up_velocity_2 * timeStep, std::max(a_required, -9.81));
        }
    }
    else
    {
        a_AEB_brake = LARGE_NUMBER;
    }
    return a_AEB_brake;
}

void ControllerHiDrive::LongitudinalControl()
{
    UpdateSideLaneSensors();
    double a_des_PI = CalculateADFAcceleration();
    double a_AEB_brake = CalculateAEBAcceleration();
    //double a_x_turn    = CalculateTurnAcceleration();
    double a_x_intersection = IntersectionAcceleration();
    // Longitudinal Control
    if (AEB_active == 1)
    {
        a_des_PI = 0;
    }

    double a_values[] = {a_des_PI, a_x_intersection, a_AEB_brake};
    double a_x_t = LARGE_NUMBER;
    for (double a_value : a_values)
    {
        if (a_value < a_x_t)
        {
			a_x_t = a_value;
		}
    }
    
    if (a_x_t == a_AEB_brake)
    {
        I_old = 0;
    }
    
    if (mode_ == ControlOperationMode::MODE_OVERRIDE)
    {
        acc = a_x_t;
    }
    
    acc_pre = acc;
    
}

double ControllerHiDrive::LateralControl()              // Lateral control from L3Pilot. The code is a mess. 
{
    t_ego_previous_previous = t_ego_previous;
    t_ego_previous          = t_ego;
    
    t_ego = object_->pos_.GetOffset();
    // --------------------------------------------------------------------------


    double max_steering_change = 1.5 * timeStep;

    double KP = 2;                                      // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.
    double KI = 0.1 * KP * (2.0 / 3.478) * 0.075;       // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.
    double KD = KP * (2.0 / 0.848) * 2.5;               // Verified as up-to-date with the 2024-04-12 version of the HiDrive motorway code.  

    double delta_t_ego = abs(t_ego - t_ego_previous_previous) / 2;
    
    if (abs(delta_t_ego) > 2)
    {
        delta_t_ego = 0;
    }
    double dy0 = t_ego;
    double dy_vehicle = dy0;
    double u_regler = -dy_vehicle;
    double PID_regler = y_regler + (KP + KI + KD) * u_regler - (KP + 2 * KD) * u_regler_1 + KD * u_regler_2;
    y_regler = PID_regler;
    double delta_steering = PID_regler - steeringWheelAngle_ego;
    if (abs(delta_steering) <= max_steering_change)
    {
		steeringWheelAngle_ego = PID_regler;
	}
    else
    {
		steeringWheelAngle_ego += max_steering_change * (delta_steering > 0 ? 1 : -1);
	}
    u_regler_2 = u_regler_1;
	u_regler_1 = u_regler;
    y_regler = steeringWheelAngle_ego;
    return steeringWheelAngle_ego;
}

void ControllerHiDrive::LogValues()
{
    // To add another value to the log, use the following syntax:
    // LOG("LOG_VALUE:Name:%f", value);
    // It must start with LOG_VALUE for the python script to pick up the value.
    
    //object_->pos_.GetLaneId();
    //object_->pos_.GetOffset();
    
    LOG("CollisionS: %f", collisionS);
    LOG("Actual S (back): %f", object_->pos_.GetS());
    LOG("Actual S (front): %f", object_->pos_.GetS() + vehicle_length);
    LOG("CollisionTime: %f", collisionTime);
    LOG("TimeEnter: %f", timeEnter);
    LOG("TimeExit: %f", timeExit);

    roadmanager::RoadLaneInfo laneInfo;
    object_->pos_.GetRoadLaneInfo(&laneInfo);
    LOG("LOG_VALUE:LaneWidth:%f", laneInfo.width);
    LOG("LOG_VALUE:Offset:%f", object_->pos_.GetOffset());
    LOG("LOG_VALUE:VehicleWidth:%f", vehicle_width);
    

    if (mode_ == ControlOperationMode::MODE_OVERRIDE) {
        LOG("LOG_VALUE:Acceleration:%f", acc);
    }
    else
    {
        LOG("LOG_VALUE:Acceleration:%f", 0.0);
    }
    LOG("LOG_VALUE:CutInDetectionTime:%f", cutInDetectionTime_);
    LOG("LOG_VALUE:Speed:%f", currentSpeed_);
    LOG("LOG_VALUE:AEB_active:%d", AEB_active || turn_AEB_active);
    Object* leadVehicle = FindLeadVehicle(false, false);
    double  x_local, y_local;
    double  dist = object_->FreeSpaceDistance(leadVehicle, &x_local, &y_local);
    LOG("LOG_VALUE:DistanceToClosestVehicle:%f", dist);
    if (dist == 0.0)
    {
        tmp_collision = 1.0;
	}
    LOG("Has collided:%f", tmp_collision);
    LOG("LOG_VALUE:Collision:%f", dist == 0.0 ? 1.0 : 0.0);
    LOG("LOG_VALUE:Time:%f", scenario_engine_->getSimulationTime());

    /*
    LOG("LOG_VALUE:Speed:%f (%s)", currentSpeed_, object_->GetName().c_str());
    LOG("LOG_VALUE:Acceleration:%f (%s)", acc, object_->GetName().c_str());
    LOG("LOG_VALUE:AEB_active:%d (%s)", AEB_active || turn_AEB_active, object_->GetName().c_str());
    Object* leadVehicle = FindLeadVehicle(false, false);
    double  x_local, y_local;
    double  dist = object_->FreeSpaceDistance(leadVehicle, &x_local, &y_local);
    LOG("LOG_VALUE:DistanceToClosestVehicle:%f (%s)", dist, object_->GetName().c_str());
    LOG("LOG_VALUE:Collision:%f (%s)", dist == 0.0 ? 1.0 : 0.0, object_->GetName().c_str());
    LOG("LOG_VALUE:Time:%f (%s)", scenario_engine_->getSimulationTime(), object_->GetName().c_str());
    */
}

double ControllerHiDrive::IntersectionAcceleration()
{
    Object* leadVehicle = FindLeadVehicle(true, false);
    if (scenario_engine_->getSimulationTime() < timeExit && leadVehicle)
    {
        bool b1 = (object_->pos_.GetS() + currentSpeed_* (timeEnter - scenario_engine_->getSimulationTime())) <= collisionS;  // The back of the vehicle is before the collision point when target enters the lane.
        bool b2 = (object_->pos_.GetS() + vehicle_length + currentSpeed_* (timeExit - scenario_engine_->getSimulationTime())) >= collisionS;	// The front of the vehicle is after the intersection when target exits the lane.

        LOG("%f", (object_->pos_.GetS() + vehicle_length + currentSpeed_ * (timeExit - scenario_engine_->getSimulationTime())));
        double safety_margin = 3; // Seconds
        bool b3 = object_->pos_.GetS() + currentSpeed_ * (timeEnter - scenario_engine_->getSimulationTime() - safety_margin) <= collisionS;  // Will the back of the vehicle have passed the collision point three seconds ago?

        double intersection_acceleration = LARGE_NUMBER;
        if (b1)
        {
                        LOG("b1 true");
        }
        if (b2)
        {
						LOG("b2 true");
		}
        if (b1 && b2) // Collision will happen if we stay on the same velocity.
        {
            TTC_turn = (collisionS - object_->pos_.GetS()-vehicle_length) / currentSpeed_;
            intersection_acceleration = CalculateTurnAEBAcceleration();
        }
        else
        {
            TTC_turn = LARGE_NUMBER;
        }
        
        if (b3 && b2 && ADF_enabled)
        {
            intersection_acceleration = min(intersection_acceleration, CalculateTurnADFAcceleration());
        }
        LOG("Found lead vehicle");
        if (b2 && BSD_enabled)
        {
            LOG("b2 true");
            if (mode_ == ControlOperationMode::MODE_ADDITIVE && scenario_engine_->getSimulationTime() >= activationTime_ && !ADF_enabled)
            {
                this->TakeControl();
            }
			intersection_acceleration = min(intersection_acceleration, CalculateBSDAcceleration());
		}
        LOG("%f", intersection_acceleration);
        return intersection_acceleration;
    }
    
    return LARGE_NUMBER;
}

double ControllerHiDrive::CalculateBSDAcceleration()
{
    double time_before_resetting = 0.3;  // The BSD system will reset its values after not being active for 0.3 seconds.
    
    double        deceleration_max    = -8;
    double        deceleration_min    = 0.0;
    double        deceleration_ramp_time = 0.5;
    double        delay        = 0.5;

    double current_time = scenario_engine_->getSimulationTime();
    if (current_time > last_time_step + time_before_resetting)
    {
		start_time = current_time;
	}
    last_time_step = current_time;
    if (current_time - start_time < delay)
    {
        return LARGE_NUMBER;
	}
    else
    {
        double ramping_time = current_time - start_time - delay;
        return deceleration_min + (deceleration_max - deceleration_min) * fmin(1, ramping_time / deceleration_ramp_time);
    }
}


/*
void ControllerHiDrive::TurnDecision()
{
    // This function is used to decide if the vehicle can turn or not. 
    // The function is called in the Step function of the ControllerHiDrive class.

    // Inputs
    double Map_direction    = 0;
    double y_ego            = 0;
    double x_ego            = 0;
    double Map_Turn_point_x = 0;
    double Map_Turn_point_y = 0;
    double Map_radius       = 0;
    double Speed_limit      = 0;

    double ObjID_0 = 0;   // If ObjID == -1 or ObjID > 1000, the object is not present.
    double relativeYnet_0;
    double relativeXnet_0;
    double Obj_vx_0;
    double Obj_vy_0;
    double numberSensors_0;

    double V2X_status;
    double error_GPS_x;
    double error_GPS_x_ego;
    double error_GPS_y;
    double error_GPS_y_ego;

    // Outputs
    double Turn_okay;
    double l_safety_gap;
    double X_net_all;
    double Y_net_all;
    double y_old_all;
    double x_old_all;
    double check_area_1;
    double check_area_2;
    double check_area_3;
    double relativeXnet_0_calc;
    double relativeYnet_0_calc;
    double X_pos_OV_1;

    double Y_pos_OV_1;
    double t_remaining;
    double dx_front_calc;
    double dv_front_calc;

    // -----
    check_area_1 = 0;
    check_area_2 = 0;
    check_area_3 = 0;

    Y_pos_OV_1 = 0;

    dx_front_calc = 250;
    dv_front_calc = -50;

    Turn_okay = 1;

    double d_s_ego;
    if (Map_direction == 1)
    {
                d_s_ego = y_ego - Map_Turn_point_y;
    }
    else
    {
                d_s_ego = x_ego - Map_Turn_point_x;
    }

    t_remaining    = d_s_ego / currentSpeed_;
    double a_y_max_curve  = 2;
    double a_x_accelerate = 2;
    double v_turn_max     = sqrt(Map_radius * a_y_max_curve);

    double t_turn         = (1 / 4 * 2 * M_PI * Map_radius) / v_turn_max;
    double t_acceleration = ((Speed_limit / 3.6) - v_turn_max) / a_x_accelerate;
    double t_gap_vehicle  = 2;

    double t_safety_gap = t_turn + t_acceleration + t_gap_vehicle;
    l_safety_gap        = max(t_safety_gap * max(currentSpeed_, v_turn_max + 10 / 3.6), 100.0);

    list<double> check_area_1_x;
    list<double> check_area_1_y;
    list<double> check_area_2_x;
    list<double> check_area_2_y;
    list<double> check_area_3_x;
    list<double> check_area_3_y;
    if (Map_direction == 1)
    {
                check_area_1_x = list(Map_Turn_point_x - l_safety_gap, Map_Turn_point_x + 10);
                check_area_1_y = list(Map_Turn_point_y + 3.5 * 1 / 2, Map_Turn_point_y - 3.5 * 1 / 2);
                check_area_2_x = list(Map_Turn_point_x - 10, Map_Turn_point_x + l_safety_gap);
                check_area_2_y = list(Map_Turn_point_y + 3.5 * 3 / 2, Map_Turn_point_y + 3.5 / 2);
                check_area_3_x = list(Map_Turn_point_x + 3.5 * 3 / 2, Map_Turn_point_x + 3.5 * 1 / 2);
                check_area_3_y = list(Map_Turn_point_y + 3.5 * 3 / 2, Map_Turn_point_y + 3.5 / 2);
    }
    else
    {
                check_area_1_x = list(Map_Turn_point_x - l_safety_gap, Map_Turn_point_x + 30);
                check_area_1_y = list(Map_Turn_point_y + 3.5 * 1 / 2, Map_Turn_point_y - 3.5 * 1 / 2);
                check_area_2_x = list(Map_Turn_point_x - 25, Map_Turn_point_x + 25);
                check_area_2_y = list(Map_Turn_point_y - 3.5 * 1 / 2, -Map_Turn_point_y - 20);
                check_area_3_x = list(Map_Turn_point_x - 25, Map_Turn_point_x + 25);
                check_area_3_y = list(Map_Turn_point_y - 3.5 * 1 / 2, -Map_Turn_point_y - 20);
    }

    relativeYnet_0_calc = relativeYnet_0;
    relativeXnet_0_calc = relativeXnet_0;

    if (ObjID_0 == -1 || ObjID_0 > 1000)
    {
                relativeXnet_0_calc = NULL;
                relativeYnet_0_calc = NULL;
                Obj_vx_0            = NULL;
                Obj_vy_0            = NULL;
    }

    // % % Zwischenspeichern der alten Werte zur Ausgabe
    double x_old_0 = relativeXnet_0_calc;
    double y_old_0 = relativeYnet_0_calc;

    if (V2X_status == 1)
    {  // 
        if (numberSensors_0 == 1)
        {
            relativeXnet_0_calc = relativeXnet_0_calc + error_GPS_x + error_GPS_x_ego;
            relativeYnet_0_calc = relativeYnet_0_calc + error_GPS_y + error_GPS_y_ego;
        }
    }

    double X_pos_OV = x_ego + relativeXnet_0_calc;
    double Y_pos_OV = y_ego + relativeYnet_0_calc;

    X_pos_OV_1 = X_pos_OV;
    Y_pos_OV_1 = Y_pos_OV_1;

    X_net_all = X_pos_OV + Obj_vx_0 * t_remaining;
    Y_net_all = Y_pos_OV + Obj_vy_0 * t_remaining;

    double X_net_all_1 = X_pos_OV + Obj_vx_0 * -0.5;
    double Y_net_all_1 = Y_pos_OV + Obj_vy_0 * -0.5;

    double check_area_1_1 = X_net_all >= min(check_area_1_x.front(), check_area_1_x.back());
    double check_area_1_2 = X_net_all <= max(check_area_1_x.front(), check_area_1_x.back());
    double check_area_1_3 = Y_net_all >= min(check_area_1_y.front(), check_area_1_y.back());
    double check_area_1_4 = Y_net_all <= max(check_area_1_y.front(), check_area_1_y.back());

    double check_area_2_1 = X_net_all >= min(check_area_2_x.front(), check_area_2_x.back());
    double check_area_2_2 = X_net_all <= max(check_area_2_x.front(), check_area_2_x.back());
    double check_area_2_3 = Y_net_all >= min(check_area_2_y.front(), check_area_2_y.back());
    double check_area_2_4 = Y_net_all <= max(check_area_2_y.front(), check_area_2_y.back());

    double check_area_3_1 = X_net_all >= min(check_area_3_x.front(), check_area_3_x.back());
    double check_area_3_2 = X_net_all <= max(check_area_3_x.front(), check_area_3_x.back());
    double check_area_3_3 = Y_net_all >= min(check_area_3_y.front(), check_area_3_y.back());
    double check_area_3_4 = Y_net_all <= max(check_area_3_y.front(), check_area_3_y.back());

    double check_area_1_1_1 = X_net_all_1 >= min(check_area_1_x.front(), check_area_1_x.back());
    double check_area_1_2_1 = X_net_all_1 <= max(check_area_1_x.front(), check_area_1_x.back());
    double check_area_1_3_1 = Y_net_all_1 >= min(check_area_1_y.front(), check_area_1_y.back());
    double check_area_1_4_1 = Y_net_all_1 <= max(check_area_1_y.front(), check_area_1_y.back());

    double check_area_2_1_1 = X_net_all_1 >= min(check_area_2_x.front(), check_area_2_x.back());
    double check_area_2_2_1 = X_net_all_1 <= max(check_area_2_x.front(), check_area_2_x.back());
    double check_area_2_3_1 = Y_net_all_1 >= min(check_area_2_y.front(), check_area_2_y.back());
    double check_area_2_4_1 = Y_net_all_1 <= max(check_area_2_y.front(), check_area_2_y.back());

    double check_area_3_1_1 = X_net_all_1 >= min(check_area_3_x.front(), check_area_3_x.back());
    double check_area_3_2_1 = X_net_all_1 <= max(check_area_3_x.front(), check_area_3_x.back());
    double check_area_3_3_1 = Y_net_all_1 >= min(check_area_3_y.front(), check_area_3_y.back());
    double check_area_3_4_1 = Y_net_all_1 <= max(check_area_3_y.front(), check_area_3_y.back());

    check_area_1 = check_area_1_1 + check_area_1_2 + check_area_1_3 + check_area_1_4 >= 4;
    check_area_2 = check_area_2_1 + check_area_2_2 + check_area_2_3 + check_area_2_4 >= 4;
    check_area_3 = check_area_3_1 + check_area_3_2 + check_area_3_3 + check_area_3_4 >= 4;

    check_area_1_1 = check_area_1_1_1 + check_area_1_2_1 + check_area_1_3_1 + check_area_1_4_1 >= 4;
    check_area_2_1 = check_area_2_1_1 + check_area_2_2_1 + check_area_2_3_1 + check_area_2_4_1 >= 4;
    check_area_3_1 = check_area_3_1_1 + check_area_3_2_1 + check_area_3_3_1 + check_area_3_4_1 >= 4;

    if (check_area_1 || check_area_1_1)
    {
                Turn_okay = 0;
    }
    if (check_area_2 || check_area_2_1)
    {
                Turn_okay = 0;
    }
    if (check_area_3 || check_area_3_1)
    {
                Turn_okay = 0;
    }

    double check_y_1 = abs(relativeYnet_0_calc) < 3.5 / 2;
    if (check_y_1 > 0)
    {
        dx_front_calc = relativeXnet_0_calc;
        dx_front_calc = dx_front_calc * -1;

        double relative_Obj_vx_all_in_lane = Obj_vx_0;
        dv_front_calc = currentSpeed_ + Obj_vx_0;
        if (abs(dx_front_calc) > 250)
        {
            dx_front_calc = 250;
            dv_front_calc = -50;
        }
    }
}


double ControllerHiDrive::CalculateTurnAcceleration(
                               bool In_turn_area,
                               double Map_stop_line_cor,
                               double Map_stop_line_y,
                               double Map_stop_line_x,
                               double a_x_ego,
                               double a_x_turn_1,
                               bool   turn_okay)
{
    double a_x_turn;
    double a_x_turn_stop;
    double delta_s;
    
    double v_ego = currentSpeed_;
    double y_ego = object_->pos_.GetY();
    double x_ego = object_->pos_.GetX();


    double turn_possible = turn_okay;
    
    if (In_turn_area)
    {
        if (Map_stop_line_cor == 1)
        {
			delta_s = y_ego - Map_stop_line_y;
		}
        else
        {
			delta_s = x_ego - Map_stop_line_x;
		}

        if (turn_possible == 0)
        {
			a_x_turn_stop = -1 * (pow(v_ego, 2) / (2 * delta_s));
            if (v_ego < 1)
            {
				a_x_turn_stop = -3;
			}
		}
        else
        {
			a_x_turn_stop = 3;
		}

		a_x_turn = a_x_turn_stop;
	}
    else
    {
		a_x_turn = a_x_turn_1;
	}
    return a_x_turn;
}

*/


/*
As we are not doing any lane changes, these two functions are not needed. The third function replaces LatralControl if lanechanges are needed, some variables might also need to be added to the hpp-file.

void ControllerHiDrive::LaneChangeCheck(double timeStep, double* trigger_LC, double* trigger_LC_abort)
{
    UpdateSideLaneSensors();
    // Inputs
    double lane_right_LC_available     = 0;
    double lane_left_LC_available      = 0;
    double MRM_status                  = 0;    // 1 if time > t_MRM, else 0, we will always have MRM_status = 0. 
    double x_ego                       = 0;
    
    // ----

    double Forced_lane_change = Forced_LC_Fct();
    double setSpeed                = setSpeed_;
    
    double x_LC_Forced_value = 99999;  // From parameter file
    double Distance_forced_lane_change = 10;     // From parameter file

    time_since_LC += timeStep;
    if (abs(status_LC) > 0)
    {
        time_since_LC = 0;
    }

    double t_limit_LC         = 15;
    double v_limit_trigger_LC = 3;
    double v_limit_LC         = 60.0f / 3.6f;

    double THW_front_vehicle_limit   = 4;
    double THW_front_right_lane_free = 6;
    double dx_min_right_lane_free    = 150;
    double THW_front_left_lane_free  = 6;
    double dx_min_left_lane_free     = 150;
    double time_out_beginning        = 13;

    double dx_limit_side  = 10;
    double THW_limit_side = 1;
    double a_req_limit    = -2;

    double dx_res = 5;

    double a_req_rear_left = 0;
    if (rearLeftObjDs - dx_res > 0)
    {
        if (rearLeftObjDv >= 0)
        {
                        a_req_rear_left = -pow(rearLeftObjDv, 2) / (2 * (rearLeftObjDs - dx_res));
        }
    }
    else
    {
        a_req_rear_left = -9.81;
    }
    double a_req_rear_right = 0;
    if (rearRightObjDs - dx_res > 0)
    {
        if (rearRightObjDv >= 0)
        {
                        a_req_rear_right = -pow(rearRightObjDv, 2) / (2 * (rearRightObjDs - dx_res));
        }
    }
    else
    {
        a_req_rear_right = -9.81;
    }
    double delta_v         = setSpeed - currentSpeed_;
    double THW_front       = 20;
    double THW_front_left  = 20;
    double THW_rear_left   = 20;
    double THW_front_right = 20;
    double THW_rear_right  = 20;
    if (currentSpeed_ > 2)
    {
        THW_front       = frontFrontObjDs / currentSpeed_;
        THW_front_left  = frontLeftObjDs / currentSpeed_;
        THW_rear_left   = rearLeftObjDs / currentSpeed_;
        THW_front_right = frontRightObjDs / currentSpeed_;
        THW_rear_right  = rearRightObjDs / currentSpeed_;
    }
    if ((THW_front_right > THW_front_right_lane_free || frontRightObjDs > dx_min_right_lane_free) && (lane_right_LC_available == 1))
    {
        *trigger_LC = -1;
    }
    if ((THW_front_left > THW_front_left_lane_free || frontLeftObjDs > dx_min_left_lane_free) && (lane_left_LC_available == 1))
    {
        *trigger_LC = 1;
    }
    if (*trigger_LC == 1)
    {
        if (delta_v > v_limit_trigger_LC && THW_front < THW_front_vehicle_limit)
        {
                        *trigger_LC = 1;
        }
        else
        {
                        *trigger_LC = 0;
        }
    }
    if (*trigger_LC == 1)
    {
        if (frontLeftObjDs < dx_limit_side || rearLeftObjDs < dx_limit_side)
        {
                        *trigger_LC = 0;
        }
        if (THW_front_left < THW_limit_side || THW_rear_left < THW_limit_side)
        {
                        *trigger_LC = 0;
        }
        if (a_req_rear_left < a_req_limit)
        {
                        *trigger_LC = 0;
        }
    }

    if (*trigger_LC == -1)
    {
        if (frontRightObjDs < dx_limit_side || rearRightObjDs < dx_limit_side)
        {
                        *trigger_LC = 0;
        }
        if (THW_front_right < THW_limit_side || THW_rear_right < THW_limit_side)
        {
                        *trigger_LC = 0;
        }
        if (a_req_rear_right < a_req_limit)
        {
                        *trigger_LC = 0;
        }
    }

    if (*trigger_LC == 1 && lane_left_LC_available == 0)
    {
        *trigger_LC = 0;
    }
    if (*trigger_LC == -1 && lane_right_LC_available == 0)
    {
        *trigger_LC = 0;
    }
    if (currentSpeed_ < v_limit_LC && MRM_status == 0)
    {
        *trigger_LC = 0;
    }
    if (*trigger_LC == 1 && MRM_status == 1)
    {
        *trigger_LC = 0;
    }
    if (time_since_LC < t_limit_LC)
    {
        *trigger_LC = 0;
    }
    if (scenario_engine_->getSimulationTime() < time_out_beginning)
    {
        *trigger_LC = 0;
    }

    if (x_ego >= x_LC_Forced_value && x_ego <= (x_LC_Forced_value + Distance_forced_lane_change))
    {
        *trigger_LC = 0;
    }
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (abs(Forced_lane_change) >= 1)
    {
        *trigger_LC = Forced_lane_change;
        if (frontRightObjDs < (dx_limit_side / 2) || rearRightObjDs < (dx_limit_side / 2))
        {
                        *trigger_LC = 0;
        }
        if (THW_front_right < (THW_limit_side / 4) || THW_rear_right < (THW_limit_side / 4))
        {
                        *trigger_LC = 0;
        }
    }

    if (scenario_engine_->getSimulationTime() > 999)
    {
        *trigger_LC_abort = 1;
        if (scenario_engine_->getSimulationTime() > (999 + timeStep))
        {
                        *trigger_LC_abort = 0;
        }
    }
}

double ControllerHiDrive::Forced_LC_Fct()
{

    // Inputs
    double x_ego         = 0;
    double LaneCountLeft = 0;
    double x_LC_Forced_value = 99999;    // From parameter file
    double distance_forced_lane_change = 10; // From parameter file

    // Outputs
    double distance_to_lane_end = 0;           // Seems like a pointless variable.

	// Distance to lane end --> required for deceleration
    if (LaneCountLeft <= 1 && x_ego >= x_LC_Forced_value && x_ego <= (x_LC_Forced_value + distance_forced_lane_change))
    {
		distance_to_lane_end = (x_LC_Forced_value + distance_forced_lane_change) - x_ego;
	}
    else
    {
		distance_to_lane_end = 999;
	}

	// Decide depending on the force lane change status whether the force lane
	// change shall be activitated.
    if (forced_lane_change == 1)
    {
        if (abs(status_LC) > 0)
        {
			forced_lane_change = 0;
		}
	}
    else if (
        x_ego > x_LC_Forced_value                                   && 
        x_ego <= (x_LC_Forced_value + distance_forced_lane_change)  && 
        max_force_lane_change == 0                                  &&
        LaneCountLeft == 0) {
        forced_lane_change = 1;
    } else {
        forced_lane_change = 0;
	}
	// Set new max status in case the current status is higher than the previous
	// one
    if (forced_lane_change > max_force_lane_change)
    {
		max_force_lane_change = forced_lane_change;
	}
    return forced_lane_change;
}

double ControllerHiDrive::LateralControl(double timeStep)              // Lateral control from L3Pilot. The code is a mess.
{
    t_ego_previous_previous = t_ego_previous;
    t_ego_previous          = t_ego;

    t_ego = object_->pos_.GetOffset();

    // --------------------------------------------------------------------------
    double trigger_SPWE;
    double trigger_abort;
    laneChangeCheck(timeStep, &trigger_SPWE, &trigger_abort);

    double max_steering_change = 1.5;

    double KP          = 2;
    double KI          = 0.1 * KP * (2.0 / 3.478) * 0.075;
    double KD          = KP * (2.0 / 0.848) * 2.5;
    double T_SPWE_Zeit = 6.0;

    double delta_t_ego = abs(t_ego - t_ego_previous_previous) / 2;

    if (abs(delta_t_ego) > 2)
    {
        delta_t_ego = 0;
    }
    if (abs(trigger_SPWE) == 1)
    {
        status_LC = trigger_SPWE;
    }
    if (counter_SPWE > 20)
    {
        status_LC = 0;
    }
    counter_SPWE = abs(status_LC) == 0 ? 0 : counter_SPWE + 0.1;
    if (trigger_abort == 1)
    {
        status_abort = 1;
    }
    if (abs(status_LC) == 1 || abs(status_abort) == 1)
    {
        if (LC_phase == 0)
        {
            LC_phase  = 1;
            offset_LC = 0;
        }
        else
        {
            offset_LC += delta_t_ego;
        }
        double d_actual_offset = t_ego;
        double target_t        = status_LC * fmin(3.5, 3.5 / T_SPWE_Zeit * counter_SPWE);
        if (status_LC == 1)
        {
            if (t_ego < 0 && counter_SPWE > 2.5)
            {
                            d_actual_offset = 3.75 + t_ego;
                            LC_phase        = 2;
            }
            if (d_actual_offset > 3.3 || counter_SPWE >= (T_SPWE_Zeit + 0.2))
            {
                            status_LC = 0;
            }
        }
        else if (status_LC == -1)
        {
            if (t_ego > 0 && counter_SPWE > 2.5)
            {
                            d_actual_offset = -3.75 + t_ego;
                            LC_phase        = 2;
            }
            //            if (d_actual_offset < -3.3 || counter_SPWE >= (T_SPWE_Zeit + 0.2))            If this can be used instead, then the two if
            //            statements can be combined
            if (abs(d_actual_offset) > 3.3 || counter_SPWE >= (T_SPWE_Zeit + 0.2))
            {
                            status_LC = 0;
            }
        }
        if (status_abort == 1)
        {
            if (abs(d_actual_offset) < 1.5)
            {
                            target_t = 0;
            }
            if (abs(d_actual_offset) < 0.25)
            {
                            status_LC    = 0;
                            status_abort = 0;
            }
        }
        double dy_vehicle     = d_actual_offset;  // Seems unneccesary. Basically just switches to a different variable name.
        double u_regler       = target_t - dy_vehicle;
        double PID_regler     = y_regler + (KP + KI + KD) * u_regler - (KP + 2 * KD) * u_regler_1 + KD * u_regler_2;
        y_regler              = PID_regler;
        double delta_steering = PID_regler - steeringWheelAngle_ego;
        if (abs(delta_steering) <= max_steering_change)
        {
            steeringWheelAngle_ego = PID_regler;
        }
        else
        {
            steeringWheelAngle_ego += max_steering_change * (delta_steering > 0 ? 1 : -1);
        }
        u_regler_2 = u_regler_1;
        u_regler_1 = u_regler;
    }
    else
    {
        double dy0            = t_ego;
        double dy_vehicle     = dy0;
        double u_regler       = -dy_vehicle;
        double PID_regler     = y_regler + (KP + KI + KD) * u_regler - (KP + 2 * KD) * u_regler_1 + KD * u_regler_2;
        y_regler              = PID_regler;
        double delta_steering = PID_regler - steeringWheelAngle_ego;
        if (abs(delta_steering) <= max_steering_change)
        {
            steeringWheelAngle_ego = PID_regler;
        }
        else
        {
            steeringWheelAngle_ego += max_steering_change * (delta_steering > 0 ? 1 : -1);
        }
        u_regler_2 = u_regler_1;
        u_regler_1 = u_regler;
        offset_LC  = 0;
    }
    y_regler = steeringWheelAngle_ego;
    return steeringWheelAngle_ego;
}

*/