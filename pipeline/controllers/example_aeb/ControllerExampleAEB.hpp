// Small AEB controller: copy this folder to add your own. See README.md, "Add a controller".
#pragma once

#include "Controller.hpp"
#include "Entities.hpp"

namespace scenarioengine
{
    class ControllerExampleAEB : public Controller
    {
    public:
        explicit ControllerExampleAEB(InitArgs* args);

        const char* GetTypeName() const override
        {
            return "ExampleAEBController";
        }
        Type GetType() const override
        {
            // Give each new controller its own number above the base.
            return static_cast<Type>(USER_CONTROLLER_TYPE_BASE + 1);
        }

        void Step(double timeStep) override;

    private:
        double speed_mps_;       // property speed_kph: cruise speed
        double ttc_trigger_s_;   // property ttc_s: brake below this time to collision
        double deceleration_;    // property deceleration_mps2
        bool   braking_;         // stays on once triggered
    };

    Controller* InstantiateControllerExampleAEB(void* args);
}  // namespace scenarioengine
