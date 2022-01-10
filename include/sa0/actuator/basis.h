#ifndef SA0_ACTUATOR_BASIS_H
#define SA0_ACTUATOR_BASIS_H
#pragma once

#include <memory>

#include "sa0/actuator/actuator.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeStepPassive>
class StepBasisSwim : public StepActuator<TypeStepPassive> {
    public:
        using Type = StepActuator<TypeStepPassive>;
        using typename Type::TypeStateVectorDynamic;
        using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
    public:
        StepBasisSwim() : StepActuator<TypeStepPassive>() {
        }
        TypeVectorStateDynamic operator()(const TypeStepObject& stepObject, const double* pState, const double& t) const {
            // Get basis
            auto basis = stepObject.basis(p_state);
            // Set dState
            TypeVectorStateDynamic dState(stepObject->stateSize());
            dState.fill(0.0);
            // Set get u view
            auto u = stepObject.x(dState);
            u = basis.transpose() * velocity * intensity;
            return dState;
        }
    public:
        using StepActuator<TypeState, TypeStepObject>::intensity;
        TypeVector velocity; // Local swim velocity
};

}

}

#endif
