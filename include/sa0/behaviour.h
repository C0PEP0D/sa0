#ifndef SAO_BEHAVIOUR_H
#define SAO_BEHAVIOUR_H
#include <memory>
#pragma once

#include <numeric>

namespace sl0 {

namespace sa0 {

// Active Agents

template<typename TypeStepActive>
class Behaviour {
    public:
        virtual void operator()(const double* state, const double& t, const TypeStepActive& stepActive) const = 0;
        virtual std::vector<typename TypeStepActive::TypeSpaceVector> positions(const double* pState, const TypeStepActive& stepActive) const {
            return {};
        }
};

}

}

#endif
