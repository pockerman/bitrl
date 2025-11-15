#ifndef STATE_AGGREGATION_ENV_BASE_H
#define STATE_AGGREGATION_ENV_BASE_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/rlenvscpp_config.h"

#ifdef GYMFCPP_DEBUG
#include <cassert>
#endif

#include <vector>
#include <string>

namespace bitrl
{

/// Forward declaration
template<typename StateTp> class TimeStep;

namespace envs{

///
/// \brief The TiledEnviromentBase class. Base class to derive
/// enviroments that discretize the continues state into a discrete number
/// of bins.
///
template<typename EnvType, typename StateTp>
class StateAggregationEnvBase
{
public:

    /*typedef EnvType env_type;
    typedef StateTp state_type;
    typedef TimeStep<state_type> time_step_type;
    typedef typename env_type::action_type action_type;


    ///
    /// \brief Destructor
    ///
    virtual ~StateAggregationEnvBase()=default;

    ///
    /// \brief make
    ///
    virtual void make();

    ///
    /// \brief reset
    /// \return
    ///
    time_step_type reset();

    ///
    /// \brief step
    /// \param action
    /// \return
    ///
    virtual time_step_type step(const action_type& action)=0;

    ///
    /// \brief create_bins. Create the bins for the variables
    ///
    virtual void create_bins()=0;

    ///
    /// \brief get_state Transforms the original state to the aggregate state
    /// \param obs. The original continuous state
    /// \return
    ///
    virtual state_type get_state(const typename env_type::state_type& obs)const=0;

    ///
    /// \brief n_states
    /// \return
    ///
    uint_t n_states()const noexcept{return n_states_;}

protected:


    ///
    /// \brief Constructor
    ///
    StateAggregationEnvBase(const std::string version, obj_t main_namespace, uint_t n_states);

    ///
    /// \brief env_ The actual environment
    ///
    env_type env_;

    ///
    /// \brief n_states_
    ///
    uint_t n_states_;

    */
};





}
}

#endif // TILED_ENV_BASE_H
