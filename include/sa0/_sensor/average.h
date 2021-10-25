#ifndef SA0_SENSOR_AVERAGE_H
#define SA0_SENSOR_AVERAGE_H
#pragma once

#include <numeric>
#include "sa0/sensor/sensor.h"

namespace sl0 {

namespace sa0 {

template<template<typename, typename, typename, typename, typename...> class TypeSensor, typename TypeSensed, typename TypeObservation, typename TypeStateMemory, typename TypeStepMemory, typename TypeStateObject, typename TypeStepObject, typename ...Args>
class SensorAverage : public SensorArray<TypeSensed, TypeObservation, TypeStateMemory, TypeStepMemory> {
    public:
        SensorAverage();
        TypeSensed operator()(const TypeStepMemory& stepMemory, const TypeStateMemory& state, const double& t) const override;
    public:
        using Sensor<TypeSensed, TypeObservation, TypeStateMemory, TypeStepMemory>::adim;
    public:
        TypeSensor<TypeSensed, TypeObservation, TypeStateObject, TypeStepObject, Args...> sensor;
};

// SensorAverage class

template<template<typename, typename, typename, typename, typename...> class TypeSensor, typename TypeSensed, typename TypeObservation, typename TypeStateMemory, typename TypeStepMemory, typename TypeStateObject, typename TypeStepObject, typename ...Args>
SensorAverage<TypeSensor, TypeSensed, TypeObservation, TypeStateMemory, TypeStepMemory, TypeStateObject, TypeStepObject, Args...>::SensorAverage() : SensorArray<TypeSensed, TypeObservation, TypeStateMemory, TypeStepMemory>() {
}

template<template<typename, typename, typename, typename, typename...> class TypeSensor, typename TypeSensed, typename TypeObservation, typename TypeStateMemory, typename TypeStepMemory, typename TypeStateObject, typename TypeStepObject, typename ...Args>
TypeSensed SensorAverage<TypeSensor, TypeSensed, TypeObservation, TypeStateMemory, TypeStepMemory, TypeStateObject, TypeStepObject, Args...>::operator()(const TypeStepMemory& stepMemory, const TypeStateMemory& state, const double& t) const {
    // Get time and memory
    auto time = stepMemory.time(state);
    auto memory = stepMemory.memory(state);
    // Compute tTot
    double tTot = time.front() - time.back();
    if(tTot > 0.0) {
        // Sum up and return
        TypeSensed sensed = TypeSensed::Zero();
        for(unsigned int i = 0; i < time.size() - 1; i++) {
            sensed += 0.5 * (sensor(stepMemory.stepObject, memory[i], time[i]) + sensor(stepMemory.stepObject, memory[i+1], time[i + 1])) * (time[i] - time[i + 1]);
        }
        sensed /= tTot;
        return sensed;
    }
    return sensor(stepMemory.stepObject, memory[0], t);
}

}

}

#endif
