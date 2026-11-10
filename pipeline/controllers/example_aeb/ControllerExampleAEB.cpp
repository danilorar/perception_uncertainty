// Drives at a fixed speed along the road and brakes when the time to collision is short.
// It is a template that shows the interface, not a validated AEB.
#include "ControllerExampleAEB.hpp"

#include <cmath>

#include "CommonMini.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerExampleAEB(void* args)
{
    return new ControllerExampleAEB(static_cast<Controller::InitArgs*>(args));
}

ControllerExampleAEB::ControllerExampleAEB(InitArgs* args)
    : Controller(args),
      speed_mps_(strtod(args->properties->GetValueStr("speed_kph")) / 3.6),
      ttc_trigger_s_(strtod(args->properties->GetValueStr("ttc_s"))),
      deceleration_(strtod(args->properties->GetValueStr("deceleration_mps2"))),
      braking_(false)
{
}

void ControllerExampleAEB::Step(double timeStep)
{
    // Targets are read during this step, so range errors reach the controller without any extra code:
    // the error hook moves them to their perceived position before Step and back afterwards.
    const double heading = object_->pos_.GetH();
    double       ttc     = LARGE_NUMBER;
    for (Object* target : entities_->object_)
    {
        if (target == nullptr || target == object_)
        {
            continue;
        }
        const double dx      = target->pos_.GetX() - object_->pos_.GetX();
        const double dy      = target->pos_.GetY() - object_->pos_.GetY();
        const double ahead   = dx * cos(heading) + dy * sin(heading);
        const double side    = -dx * sin(heading) + dy * cos(heading);
        const double gap     = ahead - (object_->boundingbox_.dimensions_.length_ / 2.0 + object_->boundingbox_.center_.x_) -
                           (target->boundingbox_.dimensions_.length_ / 2.0 - target->boundingbox_.center_.x_);
        const double width   = (object_->boundingbox_.dimensions_.width_ + target->boundingbox_.dimensions_.width_) / 2.0;
        const double closing = speed_mps_ - target->GetSpeed();
        if (ahead > 0.0 && fabs(side) < width && closing > 0.0)
        {
            ttc = MIN(ttc, MAX(gap, 0.0) / closing);
        }
    }

    braking_                  = braking_ || ttc < ttc_trigger_s_;
    const double acceleration = braking_ && speed_mps_ > 0.0 ? -deceleration_ : 0.0;
    speed_mps_                = MAX(0.0, speed_mps_ + acceleration * timeStep);

    object_->MoveAlongS(speed_mps_ * timeStep);
    object_->SetSpeed(speed_mps_);

    // The pipeline reads these two lines from esmini.log; controller.json names them under "signals".
    LOG_INFO("LOG_VALUE:AEB_active:{}", braking_ ? 1 : 0);
    LOG_INFO("LOG_VALUE:Acceleration:{}", acceleration);

    Controller::Step(timeStep);
}
