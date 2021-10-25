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

template<template<int> typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, typename TypeStepActive, typename TypeBehaviour>
class StepAgent : public TypeStepActive {
    public:
        using TypeStepActive::StateSize;
        using typename TypeStepActive::TypeStateStatic;
        using typename TypeStepActive::TypeSpaceVector;
    public:
        StepAgent(const TypeStepActive& stepActive, const std::shared_ptr<TypeBehaviour>& p_sBehaviour) : TypeStepActive(stepActive), sBehaviour(p_sBehaviour) {
        }
        TypeStateStatic operator()(const TypeRef<const TypeStateStatic>& state, const double& t) const override {
            (*sBehaviour)(state, t, *this);
            return TypeStepActive::operator()(state, t);
        }
        void update(TypeRef<TypeStateStatic> state, const double& t) override {
            TypeStepActive::update(state, t);
        }
    public:
        std::vector<TypeSpaceVector> positions(const TypeRef<const TypeStateStatic>& state) const override {
            std::vector<TypeSpaceVector> result = TypeStepActive::positions(state);
            std::vector<TypeSpaceVector> behaviourPositions = sBehaviour->positions(state, *this);
            result.insert(result.end(), behaviourPositions.begin(), behaviourPositions.end());
            return result;
        }
    public:
        std::shared_ptr<TypeBehaviour> sBehaviour;
};

template<template<int> typename TypeState, template<typename...> typename TypeRef, template<typename...> class TypeView, typename TypeStepActive, typename TypeBehaviour, typename TypeSolver>
class Agent : public ObjectStatic<TypeRef, TypeSolver, StepAgent<TypeState, TypeRef, TypeView, TypeStepActive, TypeBehaviour>> {
    public:
        using TypeStep = StepAgent<TypeState, TypeRef, TypeView, TypeStepActive, TypeBehaviour>;
    public:
        Agent(const TypeStepActive& stepActive, const std::shared_ptr<TypeBehaviour>& sBehaviour) : ObjectStatic<TypeRef, TypeSolver, TypeStep>(std::make_shared<TypeStep>(stepActive, sBehaviour)) {
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
