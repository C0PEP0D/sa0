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

template<typename TypeStepPassive>
class StepActuator {
    public:
        using TypeStateVectorDynamic = typename TypeStepPassive::TypeStateVectorDynamic;
    public:
        StepActuator() {
        }
        virtual TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const = 0;
};

template<template<typename...> class TypeView, typename TypeStepPassive, typename TypeStepActuator>
class StepActive : public TypeStepPassive {
    public:
        using TypeStepPassive::StateSize;
        using typename TypeStepPassive::TypeStateVectorDynamic;
    public:
        StepActive(const TypeStepPassive& stepPassive) : TypeStepPassive(stepPassive) {
        }
    public:
        TypeStateVectorDynamic operator()(const double* pState, const double& t) const override {
            //TypeStateStatic dState = std::accumulate(/*std::execution::par_unseq, */sStepActuators.cbegin(), sStepActuators.cend(), (*sStepPassive)(state, t), [this, state, t](TypeStateStatic result, const std::shared_ptr<TypeStepActuator>& sStepActuator){
            //    return std::move(result) + sStepActuator->operator()(state, t, *sStepPassive);
            //}); // WHY THE FUCK IS ACCUMULATE NOT WORKING !?!?
            TypeStateVectorDynamic dState = TypeStepPassive::operator()(pState, t);
            for(const auto& sStepActuator : sStepActuators) {
                dState += (*sStepActuator)(pState, t, *this);
            }
            return dState;
        }
        void update(double* pState, const double& t) override {
            TypeStepPassive::update(pState, t);
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

template<template<typename...> class TypeView, typename TypeStepPassive, typename TypeStepActuator, typename TypeSolver>
class ObjectActive : public ObjectStatic<TypeSolver, StepActive<TypeView, TypeStepPassive, TypeStepActuator>> {
    public:
        using TypeStep = StepActive<TypeView, TypeStepPassive, TypeStepActuator>;
        using Type = ObjectStatic<TypeSolver, TypeStep>;
    public:
        ObjectActive(const TypeStepPassive& stepPassive) : ObjectStatic<TypeSolver, TypeStep>(std::make_shared<TypeStep>(stepPassive)) {
        }
    public:
        // Inherited
        using Type::sSolver;
        using Type::sStep;
        using Type::state;
        using Type::t;
};

}

}

#endif
