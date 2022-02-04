#ifndef SA0_ACTUATOR_AXIS_H
#define SA0_ACTUATOR_AXIS_H
#pragma once

// lib includes
#include "sa0/active.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeStepPassive>
class StepAxisSwim : public StepActuator<TypeStepPassive> {
    public:
        using Type = StepActuator<TypeStepPassive>;
        using typename Type::TypeStateVectorDynamic;
        using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
    public:
        StepAxisSwim(double& p_velocity) : velocity(p_velocity) {
        }

        TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
            TypeStateVectorDynamic dState(stepPassive.stateSize());
            dState.fill(0.0);
            stepPassive.x(dState.data()) = stepPassive.cAxis(pState) * velocity;
            return dState;
        }
    public:
        double velocity;
};

template<typename TypeStepPassive>
class StepAxisRotate : public StepActuator<TypeStepPassive> {
    public:
        using Type = StepActuator<TypeStepPassive>;
        using typename Type::TypeStateVectorDynamic;
        using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
    public:
        StepAxisRotate(const TypeSpaceVector& p_angularVelocity) : angularVelocity(p_angularVelocity) {
        }

        TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
            TypeStateVectorDynamic dState(stepPassive.stateSize());
            dState.fill(0.0);
            stepPassive.axis(dState.data()) = angularVelocity.cross(stepPassive.cAxis(pState));
            return dState;
        }
    public:
        TypeSpaceVector angularVelocity;
};

template<typename TypeStepPassive>
class StepAxisOrient : public StepActuator<TypeStepPassive> {
    public:
        using Type = StepActuator<TypeStepPassive>;
        using typename Type::TypeStateVectorDynamic;
        using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
    public:
        StepAxisOrient(const TypeSpaceVector& p_direction, const double& p_time) : direction(p_direction), time(p_time) {
        }

        TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
            // Control theory
            // // init
            //TypeStateVectorDynamic dState(stepPassive->stateSize());
            //dState.fill(0.0);
            // // compute angular velocity
            //TypeVector axis = stepPassive.cAxis(pState);
            //double errorAngle = std::atan2(axis.cross(direction).norm(), axis.dot(direction));
            //TypeVector angularVelocityAxis = axis.cross(direction).normalized();
            //TypeVector angularVelocity = errorAngle / time * angularVelocityAxis;
            // // compute dAxis
            //stepPassive.axis(dState) = angularVelocity.cross(axis);
            // // return result
            //return dState;
            
            // Pedley model
            // // init
            TypeStateVectorDynamic dState(stepPassive.stateSize());
            dState.fill(0.0);
            // // get
            TypeSpaceVector axis = stepPassive.cAxis(pState);
            // // compute
            stepPassive.axis(dState.data()) = 1.0/(2.0 * time) * (direction - direction.dot(axis) * axis);
            // // return
            return dState;
        }
    public:
        TypeSpaceVector direction;
        double time;
};

}

}

#endif
