#ifndef SA0_SENSOR_LOCAL_H
#define SA0_SENSOR_LOCAL_H
#pragma once

#include "sa0/sensor/point.h"

namespace sl0 {

namespace sa0 {

template<template<typename, typename, typename, typename, typename...> class TypeSensorPoint, typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename ...Args>
class SensorLocalVector : public SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorLocalVector();
        TypeVector operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using Sensor<TypeVector, TypeObservation, TypeState, TypeStepObject>::adim;
    public:
        TypeVector position;
        TypeSensorPoint<TypeVector, TypeObservation, TypeState, TypeStepObject, Args...> pointSensor;
};

template<template<typename, typename, typename, typename, typename...> class TypeSensorPoint, typename TypeVector, typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename ...Args>
class SensorLocalMatrix : public SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorLocalMatrix();
        TypeMatrix operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using Sensor<TypeMatrix, TypeObservation, TypeState, TypeStepObject>::adim;
    public:
        TypeVector position;
        TypeSensorPoint<TypeMatrix, TypeObservation, TypeState, TypeStepObject, Args...> pointSensor;
};

// SensorLocalVector class

template<template<typename, typename, typename, typename, typename...> class TypeSensorPoint, typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename ...Args>
SensorLocalVector<TypeSensorPoint, TypeVector, TypeObservation, TypeState, TypeStepObject, Args...>::SensorLocalVector() : SensorArray<TypeVector, TypeObservation, TypeState, TypeStepObject>() {

}

template<template<typename, typename, typename, typename, typename...> class TypeSensorPoint, typename TypeVector, typename TypeObservation, typename TypeState, typename TypeStepObject, typename ...Args>
TypeVector SensorLocalVector<TypeSensorPoint, TypeVector, TypeObservation, TypeState, TypeStepObject, Args...>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    TypeState pointState = state;
    // Get data from state
    auto pointX = stepObject.x(pointState);
    auto basis = stepObject.basis(state);
    // Get jacobian
    pointX += basis.transpose() * position;
    TypeVector v = pointSensor(stepObject, pointState, t);
    // Return adimension result un local space
    return basis * v / adim;
}

// SensorLocalVector class

template<template<typename, typename, typename, typename, typename...> class TypeSensorPoint, typename TypeVector, typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename ...Args>
SensorLocalMatrix<TypeSensorPoint, TypeVector, TypeMatrix, TypeObservation, TypeState, TypeStepObject, Args...>::SensorLocalMatrix() : SensorArray<TypeMatrix, TypeObservation, TypeState, TypeStepObject>() {

}

template<template<typename, typename, typename, typename, typename...> class TypeSensorPoint, typename TypeVector, typename TypeMatrix, typename TypeObservation, typename TypeState, typename TypeStepObject, typename ...Args>
TypeMatrix SensorLocalMatrix<TypeSensorPoint, TypeVector, TypeMatrix, TypeObservation, TypeState, TypeStepObject, Args...>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    TypeState pointState = state;
    // Get data from state
    auto pointX = stepObject.x(pointState);
    auto basis = stepObject.basis(state);
    // Get jacobian
    pointX += basis.transpose() * position;
    TypeMatrix M = pointSensor(stepObject, pointState, t);
    // Return adimension result un local space
    return basis * M * basis.transpose() / adim;
}

}

}

#endif
