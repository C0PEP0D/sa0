#ifndef SA0_ACTUATOR_POINT_INERTIAL_H
#define SA0_ACTUATOR_POINT_INERTIAL_H
#pragma once

// lib includes
#include "sa0/active.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeStepPassive>
class StepInertialSwim : public StepActuator<TypeStepPassive> {
    public:
        using Type = StepActuator<TypeStepPassive>;
        using typename Type::TypeStateVectorDynamic;
        using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
    public:
        StepInertialSwim(const TypeSpaceVector& p_velocity) : velocity(p_velocity) {
        }

        TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
            TypeStateVectorDynamic dState(stepPassive.stateSize());
            dState.fill(0.0);
            stepPassive.u(dState.data()) = velocity / stepPassive.delay;
            return dState;
        }
    public:
        TypeSpaceVector velocity;
};

}

}

#endif
