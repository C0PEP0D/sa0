#ifndef SAO_AGENT_H
#define SAO_AGENT_H
#pragma once

// std includes
#include <memory> // shared_ptr
// module includes
#include "sl0/object.h"

namespace sl0 {

namespace sa0 {

// Active Agents

template<template<int> typename TypeVector, template<typename...> class TypeView, typename TypeStepActive, typename TypeBehaviour>
class StepAgent : public TypeStepActive {
    public:
        using TypeStepActive::StateSize;
        using typename TypeStepActive::TypeStateVectorDynamic;
        using typename TypeStepActive::TypeSpaceVector;
    public:
        StepAgent(const TypeStepActive& stepActive, const std::shared_ptr<TypeBehaviour>& p_sBehaviour) : TypeStepActive(stepActive), sBehaviour(p_sBehaviour) {
        }
        TypeStateVectorDynamic operator()(const double* pState, const double& t) const override {
            (*sBehaviour)(pState, t, *this);
            return TypeStepActive::operator()(pState, t);
        }
        void update(double* pState, const double& t) override {
            TypeStepActive::update(pState, t);
        }
    public:
        std::vector<TypeSpaceVector> positions(const double* pState) const override {
            std::vector<TypeSpaceVector> result = TypeStepActive::positions(pState);
            std::vector<TypeSpaceVector> behaviourPositions = sBehaviour->positions(pState, *this);
            result.insert(result.end(), behaviourPositions.begin(), behaviourPositions.end());
            return result;
        }
    public:
        std::shared_ptr<TypeBehaviour> sBehaviour;
};

template<template<int> typename TypeVector, template<typename...> class TypeView, typename TypeStepActive, typename TypeBehaviour, typename TypeSolver>
class Agent : public ObjectStatic<TypeSolver, StepAgent<TypeVector, TypeView, TypeStepActive, TypeBehaviour>> {
    public:
        using TypeStep = StepAgent<TypeVector, TypeView, TypeStepActive, TypeBehaviour>;
        using Type = ObjectStatic<TypeSolver, TypeStep>;
    public:
        Agent(const TypeStepActive& stepActive, const std::shared_ptr<TypeBehaviour>& sBehaviour) : ObjectStatic<TypeSolver, TypeStep>(std::make_shared<TypeStep>(stepActive, sBehaviour)) {
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
