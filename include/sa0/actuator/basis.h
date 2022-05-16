#ifndef SA0_ACTUATOR_BASIS_H
#define SA0_ACTUATOR_BASIS_H
#pragma once

// lib includes
#include "sa0/active.h"

namespace sl0 {

namespace sa0 {

// Actuators

template<typename TypeStepPassive>
class StepBasisSwim : public StepActuator<TypeStepPassive> {
	public:
		using Type = StepActuator<TypeStepPassive>;
		using typename Type::TypeStateVectorDynamic;
		using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
	public:
		StepBasisSwim(const double& p_velocity, const TypeSpaceVector& p_localDirection) : velocity(p_velocity), localDirection(p_localDirection) {
		}

		TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
			TypeStateVectorDynamic dState(stepPassive.stateSize());
			dState.fill(0.0);
			auto sAxis0 = stepPassive.cAxis(pState, 0);
			auto sAxis1 = stepPassive.cAxis(pState, 1);
			auto sAxis2 = stepPassive.cAxis2(pState);
			stepPassive.x(dState.data()) =  velocity * (
				localDirection(0) * sAxis0 + localDirection(1) * sAxis1 + localDirection(2) * sAxis2
			);
			return dState;
		}
	public:
		double velocity;
		TypeSpaceVector localDirection;
};

template<typename TypeStepPassive>
class StepBasisRotate : public StepActuator<TypeStepPassive> {
	public:
		using Type = StepActuator<TypeStepPassive>;
		using typename Type::TypeStateVectorDynamic;
		using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
	public:
		StepBasisRotate(const TypeSpaceVector& p_angularVelocity) : angularVelocity(p_angularVelocity) {
		}

		TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
			TypeStateVectorDynamic dState(stepPassive.stateSize());
			dState.fill(0.0);
			stepPassive.axis(dState.data(), 0) = angularVelocity.cross(stepPassive.cAxis(pState, 0));
			stepPassive.axis(dState.data(), 1) = angularVelocity.cross(stepPassive.cAxis(pState, 1));
			return dState;
		}
	public:
		TypeSpaceVector angularVelocity;
};

template<typename TypeStepPassive>
class StepBasisOrient : public StepActuator<TypeStepPassive> {
	public:
		using Type = StepActuator<TypeStepPassive>;
		using typename Type::TypeStateVectorDynamic;
		using TypeSpaceVector = typename TypeStepPassive::TypeSpaceVector;
	public:
		StepBasisOrient(const TypeSpaceVector& p_globalDirection, const double& p_time, const TypeSpaceVector& p_localAxis) : globalDirection(p_globalDirection), time(p_time), localAxis(p_localAxis) {
		}

		TypeStateVectorDynamic operator()(const double* pState, const double& t, const TypeStepPassive& stepPassive) const override {
			// Pedley model
			// // init
			TypeStateVectorDynamic dState(stepPassive.stateSize());
			dState.fill(0.0);
			// // get
			auto sAxis0 = stepPassive.cAxis(pState, 0);
			auto sAxis1 = stepPassive.cAxis(pState, 1);
			auto sAxis2 = stepPassive.cAxis2(pState);
			// // compute
			TypeSpaceVector angularVelocity = 1.0/(2.0 * time) * (localAxis(0) * sAxis0 + localAxis(1) * sAxis1 + localAxis(2) * sAxis2).cross(globalDirection);
			stepPassive.axis(dState.data(), 0) = angularVelocity.cross(stepPassive.cAxis(pState, 0));
			stepPassive.axis(dState.data(), 1) = angularVelocity.cross(stepPassive.cAxis(pState, 1));
			// // return
			return dState;
		}
	public:
		TypeSpaceVector globalDirection;
		double time;
		TypeSpaceVector localAxis;
};

}

}

#endif
