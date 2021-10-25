#ifndef SAO_SENSOR_AGENT_H
#define SAO_SENSOR_AGENT_H
#pragma once

// std includes
#include <memory> // shared_ptr
#include <tuple>// make_tuple, apply
// module includes
#include "sa0/agent.h"

namespace sl0 {

namespace sa0 {

// Active Agents

template<typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, typename TypeStepActive, typename TypeSensed, typename TypeOrders, typename TypeBehaviour, typename TypeSensors>
class StepAgentSensor : public StepAgent<TypeState, TypeRef> {
    public:
        StepAgentSensor(const std::shared_ptr<TypeStepActive>& sStepActive, const std::shared_ptr<TypeBehaviour>& sBehaviour, const TypeSensors& sensors);
        TypeSensed sense(const TypeRef<const TypeState>& state, const double& t) const override;
    public:
        TypeSensors sensors;
};

template<typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, template<typename> class TypeSolver, typename TypeStepActive, typename TypeSensed, typename TypeOrders, typename TypeBehaviour, typename TypeSensors>
class AgentSensor : public Object<TypeState, TypeRef, TypeSolver<StepAgentSensor<TypeState, TypeRef, TypeView, TypeStepActive, TypeSensed, TypeOrders, TypeBehaviour, TypeSensors>>, StepAgentSensor<TypeState, TypeRef, TypeView, TypeStepActive, TypeSensors, TypeSensed, TypeOrders, TypeBehaviour>> {
    public:
        using TypeStep = StepAgentSensor<TypeState, TypeRef, TypeView, TypeStepActive, TypeSensed, TypeOrders, TypeBehaviour, TypeSensors>;
    public:
        AgentSensor(const std::shared_ptr<TypeStepActive>& sStepActive, const std::shared_ptr<TypeBehaviour>& sBehaviour, const TypeSensors& sensors);
    public:
        // Inherited
        using Object<TypeState, TypeRef, TypeSolver<TypeStep>, TypeStep>::sSolver;
        using Object<TypeState, TypeRef, TypeSolver<TypeStep>, TypeStep>::sStep;
        using Object<TypeState, TypeRef, TypeSolver<TypeStep>, TypeStep>::state;
        using Object<TypeState, TypeRef, TypeSolver<TypeStep>, TypeStep>::t;
};

// StepAgent class

template<typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, typename TypeStepActive, typename TypeSensed, typename TypeOrders, typename TypeBehaviour, typename TypeSensors>
StepAgentSensor<TypeState, TypeRef, TypeView, TypeStepActive, TypeSensed, TypeOrders, TypeBehaviour, TypeSensors>::StepAgentSensor(const std::shared_ptr<TypeStepActive>& p_sStepActive, const std::shared_ptr<TypeBehaviour>& p_sBehaviour, const TypeSensors& p_sensors) : StepAgent<TypeState, TypeRef>(p_sStepActive, p_sBehaviour), sensors(p_sensors) {
}

// sense

template<typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, typename TypeStepActive, typename TypeSensed, typename TypeOrders, typename TypeBehaviour, typename TypeSensors>
TypeSensed StepAgentSensor<TypeState, TypeRef, TypeView, TypeStepActive, TypeSensed, TypeOrders, TypeBehaviour, TypeSensors>::sense(const TypeRef<const TypeState>& state, const double& t) const {
    return std::apply([this, state, t](auto&&... Obs) {
            return std::make_tuple((Obs(*this, state, t))...);
    }, sensors);
}

// Agent class

template<typename TypeState, template<typename...> class TypeRef, template<typename...> class TypeView, template<typename> class TypeSolver, typename TypeStepActive, typename TypeSensed, typename TypeOrders, typename TypeBehaviour, typename TypeSensors>
AgentSensor<TypeState, TypeRef, TypeView, TypeSolver, TypeStepActive, TypeSensed, TypeOrders, TypeBehaviour, TypeSensors>::AgentSensor(const std::shared_ptr<TypeStepActive>& sStepActive, const std::shared_ptr<TypeBehaviour>& sBehaviour, const TypeSensors& sensors) : Object<TypeState, TypeRef, TypeSolver<TypeStep>, TypeStep>(std::make_shared<TypeStep>(sStepActive, sBehaviour, sensors)) {

}

}

}

#endif
