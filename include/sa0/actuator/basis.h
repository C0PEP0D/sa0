#ifndef SA0_ACTUATOR_BASIS_H
#define SA0_ACTUATOR_BASIS_H
#pragma once

#include <memory>

#include "sa0/actuator/actuator.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeState, typename TypeStepObject, typename TypeVector>
class StepBasisSwim : public StepActuator<TypeState, TypeStepObject> {
    public:
        StepBasisSwim();
        TypeState operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const;
    public:
        using StepActuator<TypeState, TypeStepObject>::intensity;
        TypeVector velocity; // Local swim velocity
};

// ActuatorSwim class

template<typename TypeState, typename TypeStepObject, typename TypeVector>
StepBasisSwim<TypeState, TypeStepObject, TypeVector>::StepBasisSwim() : StepActuator<TypeState, TypeStepObject>() {
    velocity.fill(0.0);
}

template<typename TypeState, typename TypeStepObject, typename TypeVector>
TypeState StepBasisSwim<TypeState, TypeStepObject, TypeVector>::operator()(const TypeStepObject& stepObject, const TypeState& p_state, const double& p_t) const {
    // Get basis
    auto basis = stepObject.basis(p_state);
    // Set dState
    TypeState dState; dState.fill(0.0);
    // Set get u view
    auto u = stepObject.x(dState);
    u = basis.transpose() * velocity * intensity;
    return dState;
}

}

}

#endif
