#ifndef SA0_SENSOR_SENSOR_H
#define SA0_SENSOR_SENSOR_H
#pragma once

namespace sl0 {

namespace sa0 {

template<typename TypeObservation, typename TypeState, typename TypeStepObject>
class Observer {
    public:
        virtual TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const = 0;
};

// Sensors

template<typename TypeSensed, typename TypeObservation, typename TypeState, typename TypeStepObject>
class Sensor : public Observer<TypeObservation, TypeState, TypeStepObject> {
    public:
        Sensor();
        virtual TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override = 0;
        virtual TypeSensed operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const = 0;
    public:
        double adim;
};

template<typename TypeSensed, typename TypeObservation, typename TypeState, typename TypeStepObject>
class SensorArray : public Sensor<TypeSensed, TypeObservation, TypeState, TypeStepObject> {
    public:
        using Sensor<TypeSensed, TypeObservation, TypeState, TypeStepObject>::Sensor;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
        virtual TypeSensed operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override = 0;
};

template<typename TypeObservation, typename TypeState, typename TypeStepObject>
class SensorScalar : public Sensor<double, TypeObservation, TypeState, TypeStepObject> {
    public:
        using Sensor<double, TypeObservation, TypeState, TypeStepObject>::Sensor;
        TypeObservation observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override;
        virtual double operator()(const TypeStepObject& stepObject, const TypeState& state, const double& t) const override = 0;
};

// Sensor class

template<typename TypeSensed, typename TypeState, typename TypeStepObject, typename TypeVector>
Sensor<TypeSensed, TypeState, TypeStepObject, TypeVector>::Sensor() : adim(1.0) {
}

// SensorArray class

template<typename TypeSensed, typename TypeObservation, typename TypeState, typename TypeStepObject>
TypeObservation SensorArray<TypeSensed, TypeObservation, TypeState, TypeStepObject>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    const auto sensed = operator()(stepObject, state, t);
    return TypeObservation(sensed.data(), sensed.data() + sensed.size());
}

template<typename TypeObservation, typename TypeState, typename TypeStepObject>
TypeObservation SensorScalar<TypeObservation, TypeState, TypeStepObject>::observe(const TypeStepObject& stepObject, const TypeState& state, const double& t) const {
    const double sensed = operator()(stepObject, state, t);
    return TypeObservation(1, sensed);
}

}

}

#endif
