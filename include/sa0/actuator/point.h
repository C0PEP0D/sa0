#ifndef SA0_ACTUATOR_POINT_H
#define SA0_ACTUATOR_POINT_H
#pragma once

// lib includes
#include "sa0/active.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeState, template<typename...> class TypeRef, typename TypeStepPassive, typename TypeVector>
class StepPointSwim : public StepActuator<TypeState, TypeRef, TypeStepPassive> {
    public:
        StepPointSwim(const TypeRef<const TypeVector>& p_velocity) : velocity(p_velocity) {
        }

        TypeState operator()(const TypeRef<const TypeState>& state, const double& t, const TypeStepPassive& stepPassive) const override {
            TypeState dState = TypeState::Zero();
            stepPassive.x(dState) = velocity;
            return dState;
        }
    public:
        TypeVector velocity;
};

}

}

#endif
