#ifndef SA0_ACTUATOR_AXIS_H
#define SA0_ACTUATOR_AXIS_H
#pragma once

// lib includes
#include "sa0/active.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeState, template<typename...> class TypeRef, typename TypeStepPassive>
class StepAxisSwim : public StepActuator<TypeState, TypeRef, TypeStepPassive> {
    public:
        StepAxisSwim(double& p_velocity) : velocity(p_velocity) {
        }

        TypeState operator()(const TypeRef<const TypeState>& state, const double& t, const TypeStepPassive& stepPassive) const override {
            TypeState dState = TypeState::Zero();
            stepPassive.x(dState) = stepPassive.cAxis(state) * velocity;
            return dState;
        }
    public:
        double velocity;
};

template<typename TypeState, template<typename...> class TypeRef, typename TypeStepPassive, typename TypeVector>
class StepAxisRotate : public StepActuator<TypeState, TypeRef, TypeStepPassive> {
    public:
        StepAxisRotate(const TypeRef<const TypeVector>& p_angularVelocity) : angularVelocity(p_angularVelocity) {
        }

        TypeState operator()(const TypeRef<const TypeState>& state, const double& t, const TypeStepPassive& stepPassive) const override {
            TypeState dState = TypeState::Zero();
            stepPassive.axis(dState) = angularVelocity.cross(stepPassive.cAxis(state));
            return dState;
        }
    public:
        TypeVector angularVelocity;
};

template<typename TypeState, template<typename...> class TypeRef, typename TypeStepPassive, typename TypeVector>
class StepAxisOrient : public StepActuator<TypeState, TypeRef, TypeStepPassive> {
    public:
        StepAxisOrient(const TypeRef<const TypeVector>& p_direction, const double& p_time) : direction(p_direction), time(p_time) {
        }

        TypeState operator()(const TypeRef<const TypeState>& state, const double& t, const TypeStepPassive& stepPassive) const override {
            // Control engineering
            // // init
            //TypeState dState = TypeState::Zero();
            // // compute angular velocity
            //TypeVector axis = stepPassive.cAxis(state);
            //double errorAngle = std::atan2(axis.cross(direction).norm(), axis.dot(direction));
            //TypeVector angularVelocityAxis = axis.cross(direction).normalized();
            //TypeVector angularVelocity = errorAngle / time * angularVelocityAxis;
            // // compute dAxis
            //stepPassive.axis(dState) = angularVelocity.cross(axis);
            // // return result
            //return dState;
            
            // Pedley model
            // // init
            TypeState dState = TypeState::Zero();
            // // get
            TypeVector axis = stepPassive.cAxis(state);
            // // compute
            stepPassive.axis(dState) = 1.0/(2.0 * time) * (direction - direction.dot(axis) * axis);
            // // return
            return dState;
        }
    public:
        TypeVector direction;
        double time;
};

}

}

#endif
