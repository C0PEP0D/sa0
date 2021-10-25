// std includes
#include <iostream> // cout, endl
#include <memory> // shared_ptr
#include <tuple>
#include <vector>
// thirdparties includes
#include <Eigen/Dense>
// lib includes
// // s0s
#include "s0s/runge_kutta_fehlberg.h"
#include "sl0/point.h"
// // sa0
#include "sa0/active.h"
#include "sa0/actuator/point.h"
// simple includes
#include "flow.h"

using TypeScalar = double;
template<typename ...Args>
using TypeContainer = std::vector<Args...>;
// Space
template<int StateSize>
using TypeState = Eigen::Matrix<TypeScalar, StateSize, 1>;
constexpr unsigned int DIM = 3;
using TypeVector = Eigen::Matrix<TypeScalar, DIM, 1>;
// Ref and View
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
// Solver
using TypeSolver = s0s::SolverRungeKuttaFehlberg;

int main () { 
    TypeVector us = TypeVector::Constant(0.0);
    us[0] = 1.0;
    TypeVector x0 = TypeVector::Constant(1.0);
    double t0 = 0.0;
    double dt = 1e-0;
    double tEnd = 1.0;
    unsigned int nt = std::round((tEnd - t0) / dt);
    // Create activePoint
    sl0::sa0::ObjectActive<TypeState, TypeRef, TypeView, TypeStepPassive, TypeStepActuator, TypeSolver> activePoint(TypeStepPassive(std::make_shared<Flow>()));
    std::shared_ptr<TypeStepPointSwim> sStepPointSwim = std::make_shared<TypeStepPointSwim>(us);
    activePoint.sStep->register_actuator(sStepPointSwim);
    // Set initial state
    activePoint.sStep->x(activePoint.state) = x0;
    activePoint.t = t0;
    // Computation
    for(std::size_t i = 0; i < nt; i++) {
        activePoint.update(dt);
    }
    // out
    std::cout << "\n";
    std::cout << "activePoint advected and swimming in an exponential flow, t = " << activePoint.t << "\n";
    std::cout << "\n";
    std::cout << "activePoint position : " << "\n" << activePoint.sStep->x(activePoint.state) << "\n";
    std::cout << std::endl;
}
