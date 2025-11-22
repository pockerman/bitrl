#ifndef MULTI_ARMED_BANDITS_H
#define MULTI_ARMED_BANDITS_H

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/utils/maths/statistics/distributions/bernoulli_dist.h"

#include <vector>
#include <string>
#include <any>
#include <unordered_map>


namespace bitrl{
namespace envs{
namespace bandits{
	
///
/// \brief struct MultiArmedBanditsSpace specifies the
/// MultiArmedBandits state-action space
///	
struct  MultiArmedBanditsSpace
{
	
	///
	/// \brief The type describing the state space for the environment
	///
	typedef Null state_space;
	
	///
	/// \brief The type of the state
	///
	typedef Null state_type;
	
	///
	/// \brief The type of the action space for the environment
	///
	typedef Null action_space;

    ///
	/// \brief The type of the action to be undertaken in the environment
	///
    typedef uint_t action_type;
	
	
};
	
///
/// \brief class MultiArmedBandits. Environment for simulating armed-bandits
/// The bandits are represented as Bernoulli distribution. At each step
/// only one bandit can be executed
///
class MultiArmedBandits final: public EnvBase<TimeStep<bool>, MultiArmedBanditsSpace>{
	
public:
	
	///
    /// \brief name
    ///
    static const std::string name;
	
	///
	/// \brief The base type
	///
	typedef EnvBase<TimeStep<bool>, MultiArmedBanditsSpace> base_type;
	
	///
	/// \brief The time step type we return every time a step in the
	/// environment is performed
	///
    typedef typename base_type::time_step_type time_step_type;
	
	///
	/// \brief The type describing the state space for the environment
	///
	typedef typename base_type::state_space_type state_space_type;
	
	///
	/// \brief The type of the action space for the environment
	///
	typedef typename base_type::action_space_type action_space_type;

    ///
	/// \brief The type of the action to be undertaken in the environment
	///
    typedef typename base_type::action_type action_type;
	
	///
	/// \brief The type of the action to be undertaken in the environment
	///
    typedef typename base_type::state_type state_type;

	using base_type::reset;
	
	///
	/// \brief MultiArmedBandits Constructor
	///
	MultiArmedBandits();
	
	///
	/// \brief make. Builds the environment.
    /// \param version. the version of the environment to build
	/// \param options. Options to use for building the environment.
	/// Concrete classes may choose to hold a copy 
	///
    virtual void make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& options)override final;

    ///
	/// \brief close the environment
	///
    virtual void close()override final;
	
	/// 
	/// \brief Reset the environment
	/// \param seed. The seed to use for resetting the environment
	/// \param options. Options to use for resetting the environment.
	///
    virtual time_step_type reset(uint_t seed,
                                 const std::unordered_map<std::string, std::any>& options)override final;
								 

	///
	/// \brief step in the environment by performing the given action
    /// \param action. The action to execute in the environment 
	/// \return An instance of time_step_type 
    virtual time_step_type step(const action_type& action)override final;
	
	///
	/// \brief Return the number of actions
	///
	uint_t n_actions()const noexcept{return bandits_.size();}
	
	///
	/// \brief Returns the sucees reward
	///
	real_t success_reward()const noexcept{return success_reward_;}
	
	///
	/// \brief Returns the fail reward
	///
	real_t fail_reward()const noexcept{return fail_reward_;}
	
private:
	
	///
	/// \brief The seed to use
	///
	uint_t seed_;
	
	///
	/// \brief The success reward
	///
	real_t success_reward_;
	
	///
	/// \brief The reward to return on fail. 
	/// Default is zero
	///
	real_t fail_reward_;
	
	///
	/// \brief Every bandit is represented as a Bernoulli distribution
	///
	std::vector<utils::maths::stats::BernoulliDist> bandits_;
	
};
	
}	
}
}



#endif