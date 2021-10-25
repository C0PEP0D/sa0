#ifndef SA0_SENSOR_POINT_H
#define SA0_SENSOR_POINT_H
#pragma once

#include <memory>

#include "sa0/sensor/sensor.h"

namespace sl0 {

namespace sa0 {

// Basic theoritical sensors

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject>
class SensorDirection : public SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorDirection();
        TypeVector operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>::adim;
        TypeVector direction;
};

// Basic flow sensors

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeFlow>
class SensorJacobian : public SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorJacobian();
        TypeMatrix operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::adim;
        std::shared_ptr<TypeFlow> sFlow;
};

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeFlow>
class SensorStrain : public SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorStrain();
        TypeMatrix operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::adim;
        std::shared_ptr<TypeFlow> sFlow;
};

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeFlow>
class SensorAcceleration : public SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorAcceleration();
        TypeVector operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>::adim;
        std::shared_ptr<TypeFlow> sFlow;
};

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeFlow>
class SensorVelocity : public SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorVelocity();
        TypeVector operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>::adim;
        std::shared_ptr<TypeFlow> sFlow;
};

// Basic chemical sensors

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeChemical>
class SensorQ : public SensorScalar<TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorQ();
        double operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::adim;
        std::shared_ptr<TypeChemical> sChemical;
        double radius;
};

// SensorDirection class

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject>
SensorDirection<TypeVector, TypeObservation, TypeState, TypeStepObject>::SensorDirection() : SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>() {
    direction.fill(0.0);
}

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject>
TypeVector SensorDirection<TypeVector, TypeObservation, TypeState, TypeStepObject>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    // Return direction in local space
    return direction / adim;
}

// SensorJacobian class

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeFlow>
SensorJacobian<TypeMatrix, TypeObservation, TypeState, TypeStepObject, TypeVector, TypeFlow>::SensorJacobian() : SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>() {

}

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeFlow>
TypeMatrix SensorJacobian<TypeMatrix, TypeObservation, TypeState, TypeStepObject, TypeVector, TypeFlow>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    // Get data from state
    const auto x = stepObject.x(state);
    // Get jacobian
    TypeMatrix J = sFlow->getJacobian(x, t);
    // Return adimension result un local space
    return J / adim;
}

// SensorStrain class

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeFlow>
SensorStrain<TypeMatrix, TypeObservation, TypeState, TypeStepObject, TypeVector, TypeFlow>::SensorStrain() : SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>() {

}

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeFlow>
TypeMatrix SensorStrain<TypeMatrix, TypeObservation, TypeState, TypeStepObject, TypeVector, TypeFlow>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    // Get data from state
    const auto x = stepObject.x(state);
    // Get jacobian
    TypeMatrix J = sFlow->getJacobian(x, t);
    // Return adimension result
    return 0.5 * (J + J.transpose()) / adim;
}

// SensorAcceleration class

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeFlow>
SensorAcceleration<TypeVector, TypeObservation, TypeState, TypeStepObject, TypeFlow>::SensorAcceleration() : SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>() {

}

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeFlow>
TypeVector SensorAcceleration<TypeVector, TypeObservation, TypeState, TypeStepObject, TypeFlow>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    // Get data from state
    const auto x = stepObject.x(state);
    // Get strain
    TypeVector acc = sFlow->getAcceleration(x, t);
    // Return adimension result un local space
    return acc / adim;
}

// SensorVelocity class

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeFlow>
SensorVelocity<TypeVector, TypeObservation, TypeState, TypeStepObject, TypeFlow>::SensorVelocity() : SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>() {

}

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeFlow>
TypeVector SensorVelocity<TypeVector, TypeObservation, TypeState, TypeStepObject, TypeFlow>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    // Get data from state
    const auto x = stepObject.x(state);
    // Get strain
    TypeVector u = sFlow->getVelocity(x, t);
    // Return adimension result un local space
    return u / adim;
}

// SensorQ class

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeChemical>
SensorQ<TypeObservation, TypeState, TypeStepObject, TypeVector, TypeChemical>::SensorQ() : SensorScalar<TypeState, TypeStepObject, TypeVector>() {

}

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector, typename TypeChemical>
double SensorQ<TypeObservation, TypeState, TypeStepObject, TypeVector, TypeChemical>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    // Get data from state
    const auto x = stepObject.x(state);
    // Get q // TODO should be dealt inside sChemical
    double q = 0.0;
    auto neighbours = sChemical->spacePartition.getNeighbours(x);
    for(const size_t& c : neighbours) {
        TypeVector cCenter = sChemical->getCellCenter(c);
        TypeVector r = cCenter - x;
        if(r.norm() < radius) {
             sChemical->data.cellArrays[0][c];
        }
    }
    return q / adim;
}

}

}

#endif
