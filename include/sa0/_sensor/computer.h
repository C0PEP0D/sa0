#ifndef SA0_SENSOR_COMPUTER_H
#define SA0_SENSOR_COMPUTER_H
#pragma once

#include <memory>

#include "sa0/sensor/sensor.h"

namespace sl0 {

namespace sa0 {

// Computing sensors

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
class SensorDot : public SensorScalar<TypeObservation, TypeState, TypeStepObject> {
    public:
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::SensorScalar;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const;
        double operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::adim;
        std::shared_ptr<SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>> sA;
        std::shared_ptr<SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>> sB;
};

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeMatrix>
class SensorMult : public SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject> {
    public:
        using SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>::SensorArray;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const;
        TypeVector operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        std::shared_ptr<SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>> sA;
        std::shared_ptr<SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>> sB;
        using SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>::adim;
};

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject>
class SensorTranspose : public SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject> {
    public:
        using SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::SensorArray;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const;
        TypeMatrix operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        std::shared_ptr<SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>> sSensor;
        using SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::adim;
};

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
class SensorNorm : public SensorScalar<TypeObservation, TypeState, TypeStepObject> {
    public:
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::SensorScalar;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const;
        double operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        std::shared_ptr<SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>> sSensor;
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::adim;
};

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
class SensorAxis : public SensorScalar<TypeObservation, TypeState, TypeStepObject> {
    public:
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::SensorScalar;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const;
        double operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        size_t axis;
        std::shared_ptr<SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>> sSensor;
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::adim;
};

// SensorDot

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
double SensorDot<TypeObservation, TypeState, TypeStepObject, TypeVector>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    const auto sensedA = (*sA)(stepObject, state, t);
    const auto sensedB = (*sB)(stepObject, state, t);
    return sensedA.dot(sensedB) / adim;
}

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
TypeObservation SensorDot<TypeObservation, TypeState, TypeStepObject, TypeVector>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    return TypeObservation(1, operator()(stepObject, state, t));
}

// SensorMult

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeMatrix>
TypeVector SensorMult<TypeVector, TypeObservation, TypeState, TypeStepObject, TypeMatrix>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    auto sensedA = (*sA)(stepObject, state, t);
    auto sensedB = (*sB)(stepObject, state, t);
    return sensedA * sensedB / adim;
}

template<typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeMatrix>
TypeObservation SensorMult<TypeVector, TypeObservation, TypeState, TypeStepObject, TypeMatrix>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    const auto sensed =  operator()(stepObject, state, t);
    return TypeObservation(sensed.begin(), sensed.end());
}

// SensorTranspose

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject>
TypeMatrix SensorTranspose<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    return (*sSensor)(stepObject, state, t).transpose();
}

template<typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject>
TypeObservation SensorTranspose<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    const auto sensed = operator()(stepObject, state, t).reshaped();
    return TypeObservation(sensed.begin(), sensed.end());
}

// SensorNorm

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
double SensorNorm<TypeObservation, TypeState, TypeStepObject, TypeVector>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    auto sensed = (*sSensor)(stepObject, state, t);
    return sensed.norm() / adim;
}

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
TypeObservation SensorNorm<TypeObservation, TypeState, TypeStepObject, TypeVector>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    return TypeObservation(1, operator()(stepObject, state, t));
}

// SensorAxis

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
double SensorAxis<TypeObservation, TypeState, TypeStepObject, TypeVector>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    auto sensed = (*sSensor)(stepObject, state, t);
    return sensed(axis) / adim;
}

template<typename TypeObservation, typename TypeState, typename TypeStepObject, typename TypeVector>
TypeObservation SensorAxis<TypeObservation, TypeState, TypeStepObject, TypeVector>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    return TypeObservation(1, operator()(stepObject, state, t));
}

}

}

#endif
