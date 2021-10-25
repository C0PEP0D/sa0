#ifndef SAO_ACTIVE_H
#define SAO_ACTIVE_H
#pragma once

// std includes
#include <vector>
#include <numeric> // reduce
#include <memory> // shared_ptr
//#include <execution> // parallel algorithms
// module includes
#include "sl0/object.h"

namespace sl0 {

namespace sa0 {

// Active Agents

template<typename TypeState, template<typename...> class TypeRef, typename TypeStepPassive>
class StepActuator {
    public:
        StepActuator() {
        }
        virtual TypeState operator()(const TypeRef<const TypeState>& state, const double& t, const TypeStepPassive& stepPassive) const = 0;
};

template<template<int> typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, typename TypeStepPassive, typename TypeStepActuator>
class StepActive : public TypeStepPassive {
    public:
        using TypeStepPassive::StateSize;
        using typename TypeStepPassive::TypeStateStatic;
    public:
        StepActive(const TypeStepPassive& stepPassive) : TypeStepPassive(stepPassive) {
        }
    public:
        TypeStateStatic operator()(const TypeRef<const TypeStateStatic>& state, const double& t) const override {
            //TypeStateStatic dState = std::accumulate(/*std::execution::par_unseq, */sStepActuators.cbegin(), sStepActuators.cend(), (*sStepPassive)(state, t), [this, state, t](TypeStateStatic result, const std::shared_ptr<TypeStepActuator>& sStepActuator){
            //    return std::move(result) + sStepActuator->operator()(state, t, *sStepPassive);
            //}); // WHY THE FUCK IS ACCUMULATE NOT WORKING !?!?
            TypeStateStatic dState = TypeStepPassive::operator()(state, t);
            for(const auto& sStepActuator : sStepActuators) {
                dState += (*sStepActuator)(state, t, *this);
            }
            return dState;
        }
        void update(TypeRef<TypeStateStatic> state, const double& t) override {
            TypeStepPassive::update(state, t);
        }
    public:
        // registering
        void register_actuator(std::shared_ptr<TypeStepActuator> sStep) {
            sStepActuators.push_back(sStep);
        }
        void unregister_actuator(const unsigned int& actuatorIndex) {
            sStepActuators.push_back(sStepActuators.begin() + actuatorIndex);
        }
    public:
        std::vector<std::shared_ptr<TypeStepActuator>> sStepActuators;
};

template<template<int> typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, typename TypeStepPassive, typename TypeStepActuator, typename TypeSolver>
class ObjectActive : public ObjectStatic<TypeRef, TypeSolver, StepActive<TypeState, TypeRef, TypeView, TypeStepPassive, TypeStepActuator>> {
    public:
        using TypeStep = StepActive<TypeState, TypeRef, TypeView, TypeStepPassive, TypeStepActuator>;
    public:
        ObjectActive(const TypeStepPassive& stepPassive) : ObjectStatic<TypeRef, TypeSolver, TypeStep>(std::make_shared<TypeStep>(stepPassive)) {
        }
    public:
        // Inherited
        using ObjectStatic<TypeRef, TypeSolver, TypeStep>::sSolver;
        using ObjectStatic<TypeRef, TypeSolver, TypeStep>::sStep;
        using ObjectStatic<TypeRef, TypeSolver, TypeStep>::state;
        using ObjectStatic<TypeRef, TypeSolver, TypeStep>::t;
};

}

}

#endif
