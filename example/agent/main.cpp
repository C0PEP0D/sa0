// std includes
#include <iostream> // cout, endl
#include <memory> // shared_ptr
#include <vector>
#include <tuple>
// thirdparties includes
#include <Eigen/Dense>
// lib includes
// // s0s
#include "s0s/runge_kutta_fehlberg.h"
// // sl0
#include "sl0/object.h"
#include "sl0/point.h"
// // sa0
#include "sa0/active.h"
#include "sa0/actuator/point.h"
#include "sa0/agent.h"
#include "sa0/behaviour.h"
// simple includes
#include "flow.h"

using TypeScalar = double;
template<typename ...Args>
using TypeContainer = std::vector<Args...>;
// Space
constexpr unsigned int DIM = 3;
using TypeVector = Eigen::Matrix<TypeScalar, DIM, 1>;
// Ref and View
template<int StateSize>
using TypeState = Eigen::Matrix<TypeScalar, StateSize, 1>;
template<typename ...Args>
using TypeRef = Eigen::Ref<Args...>;
template<typename ...Args>
using TypeView = Eigen::Map<Args...>;
// Point
using TypeStepPoint = sl0::StepPoint<TypeState, DIM, TypeRef, TypeView, Flow>;
// Choose passive
using TypeStepPassive = TypeStepPoint;
// Active
using TypeStepActuator = sl0::sa0::StepActuator<TypeStepPassive::TypeStateStatic, TypeRef, TypeStepPassive>;
using TypeStepPointSwim = sl0::sa0::StepPointSwim<TypeStepPassive::TypeStateStatic, TypeRef, TypeStepPassive, TypeVector>;
using TypeStepActive = sl0::sa0::StepActive<TypeState, TypeRef, TypeView, TypeStepPassive, TypeStepActuator>;
// Agent
class BehaviourCustom : public sl0::sa0::Behaviour<TypeStepActive::TypeStateStatic, TypeRef, TypeStepActive> {
    public:
        std::shared_ptr<Flow> sFlow;
    public:
        BehaviourCustom(const std::shared_ptr<Flow> p_sFlow) : sFlow(p_sFlow) {
        }
        void operator()(const TypeRef<const TypeStepActive::TypeStateStatic>& state, const double& t, const TypeStepActive&  stepActive) const override {
            dynamic_cast<TypeStepPointSwim&>(*stepActive.sStepActuators[0]).velocity = sFlow->getVelocity(stepActive.cX(state), t);
        }
};
using TypeBehaviour = BehaviourCustom;
using TypeStep = sl0::sa0::StepAgent<TypeState, TypeRef, TypeView, TypeStepActive, TypeBehaviour>;
// Solver
using TypeSolver = s0s::SolverRungeKuttaFehlberg;

int main () { 
    TypeVector us = TypeVector::Constant(0.0);
    TypeVector x0 = TypeVector::Constant(1.0);
    double t0 = 0.0;
    double dt = 1e-3;
    double tEnd = 0.5;
    unsigned int nt = std::round((tEnd - t0) / dt);
    // Create flow
    std::shared_ptr<Flow> sFlow = std::make_shared<Flow>();
    // Create agent
    sl0::sa0::Agent<TypeState, TypeRef, TypeView, TypeStepActive, TypeBehaviour, TypeSolver> agent(TypeStepActive(TypeStepPassive(sFlow)), std::make_shared<TypeBehaviour>(sFlow));
    std::shared_ptr<TypeStepPointSwim> sStepPointSwim = std::make_shared<TypeStepPointSwim>(us);
    agent.sStep->register_actuator(sStepPointSwim);
    // Set initial state
    agent.sStep->x(agent.state) = x0;
    agent.t = t0;
    // Computation
    for(std::size_t i = 0; i < nt; i++) {
        agent.update(dt);
    }
    // out
    std::cout << "\n";
    std::cout << "agent advected and swimming in an exponential flow, t = " << agent.t << ", x = " << "\n";
    std::cout << "\n";
    std::cout << "agent position : " << "\n" << agent.sStep->x(agent.state) << "\n";
    std::cout << std::endl;
}
