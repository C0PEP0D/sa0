#ifndef SA0_SENSOR_TIME_H
#define SA0_SENSOR_TIME_H
#pragma once

#include <memory>

#include "sa0/sensor/sensor.h"

namespace sl0 {

namespace sa0 {

// Basic time sensors

template<typename TypeObservation, typename TypeState, typename TypeStepObject>
class SensorTime : public SensorScalar<TypeObservation, TypeState, TypeStepObject> {
    public:
        SensorTime();
        double operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
    public:
        using SensorScalar<TypeObservation, TypeState, TypeStepObject>::adim;
};

// SensorQ class

template<typename TypeObservation, typename TypeState, typename TypeStepObject>
SensorTime<TypeObservation, TypeState, TypeStepObject>::SensorTime() : SensorScalar<TypeObservation, TypeState, TypeStepObject>() {

}

template<typename TypeObservation, typename TypeState, typename TypeStepObject>
double SensorTime<TypeObservation, TypeState, TypeStepObject>::operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    return t / adim;
}

}

}

#endif
