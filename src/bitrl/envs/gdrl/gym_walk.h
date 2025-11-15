/*
 * GymWalk environment from
 * <a href="https://github.com/mimoralea/gym-walk2">gym_walk</a>
 * 
 * 
 * 
 */ 

#ifndef GYM_WALK_H
#define GYM_WALK_H

#include "rlenvs/rlenvscpp_config.h"
#include "rlenvs/rlenvs_types_v2.h"
#include "rlenvs/envs/time_step.h"
#include "rlenvs/envs/env_types.h"
#include "rlenvs/envs/env_base.h"
#include "rlenvs/extern/nlohmann/json/json.hpp"

#include <boost/noncopyable.hpp>
#include <vector>
#include <tuple>
#include <string>
#include <any>
#include <unordered_map>
#include <memory>

#ifdef RLENVSCPP_DEBUG
#include <cassert>
#endif

namespace rlenvscpp{
namespace envs{
namespace gdrl{


///
/// \brief class GymWalk. Interface for the GymWalk environment
///
template<uint_t state_size>
class GymWalk final: public EnvBase<TimeStep<uint_t>, 
                                    ScalarDiscreteEnv<state_size, 2, 0, 0>
									>
{
public:
	
	///
    /// \brief name
    ///
    static  const std::string name;
	
	///
	/// \brief The URI for accessing the environment
	///
	static const std::string URI;
	
	typedef EnvBase<TimeStep<uint_t>, 
                                    ScalarDiscreteEnv<state_size, 2, 0, 0>
									> base_type;
									
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
	/// \brief The type of the state
	///
	typedef typename base_type::state_type state_type; 
									
	///
    /// \brief dynamics_t
    ///
    typedef std::vector<std::tuple<real_t, uint_t, real_t, bool>> dynamics_t;								
									
    ///
	/// \brief Constructor
	///
    GymWalk(const RESTApiServerWrapper& api_server);
	
	///
	/// \brief Constructor
	///
	GymWalk(const RESTApiServerWrapper& api_server, 
			const uint_t cidx);
			
	///
	/// \brief copy ctor
	///
	GymWalk(const GymWalk& other);

    ///
    /// \brief make. Builds the environment. Optionally we can choose if the
    /// environment will be slippery
    ///
    virtual void make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& options) override final;
					  
					  
	///
	/// \brief
	///
	virtual bool is_alive()const override final;
	
	///
    /// \brief close the environment
    ///
    virtual void close() override final;
	
	///
    /// \brief step
    /// \param action
    /// \return
    ///
    virtual time_step_type step(const action_type& action) override final;
	
	/// 
	/// \brief Reset the environment
	///
    virtual time_step_type reset(uint_t seed,
                                 const std::unordered_map<std::string, std::any>& options)override final;
								 
								 
	///
	/// \brief Create a new copy of the environment with the given
	/// copy index
	///
	virtual std::unique_ptr<base_type> make_copy(uint_t cidx)const override final;


    ///
    /// \brief n_states. Returns the number of states
    ///
    uint_t n_states()const noexcept{ return state_space_type::size; }

    ///
    /// \brief n_actions. Returns the number of actions
    ///
    uint_t n_actions()const noexcept{return action_space_type::size;}


private:
	
    ///
    /// \brief build the dynamics from response
    ///
    dynamics_t build_dynamics_from_response_(const nlohmann::json& response)const;

    ///
    /// \brief Handle the reset response from the environment server
    ///
    time_step_type create_time_step_from_response_(const nlohmann::json& response) const;
	
	
	RESTApiServerWrapper api_server_;
};

template<uint_t state_size>
const std::string GymWalk<state_size>::name = "GymWalk";
const std::string GymWalk<state_size>::URI =  "/gdrl/gym-walk-env";


template<uint_t state_size>
GymWalk<state_size>::GymWalk(const RESTApiServerWrapper& api_server)
:
EnvBase<TimeStep<uint_t>, 
		ScalarDiscreteEnv<state_size, 2, 0, 0>
	   >(0, GymWalk<state_size>::name),
api_server_(api_server_)
{
	api_server_.register_if_not(GymWalk<state_size>::name, GymWalk<state_size>::URI);
}

template<uint_t state_size>
GymWalk<state_size>::GymWalk(const RESTApiServerWrapper& api_server, 
			                 const uint_t cidx)
:
EnvBase<TimeStep<uint_t>, 
		ScalarDiscreteEnv<state_size, 2, 0, 0>
	   >(cidx, "GymWalk"),
api_server_(api_server)
{
	api_server_.register_if_not(GymWalk<state_size>::name, GymWalk<state_size>::URI);
}


template<uint_t state_size>
GymWalk<state_size>::GymWalk(const GymWalk<state_size>& other)
:
EnvBase<TimeStep<uint_t>, 
		ScalarDiscreteEnv<state_size, 2, 0, 0>
	   >(other),
api_server_(other.api_server_)
{}

template<uint_t state_size>
typename GymWalk<state_size>::dynamics_t
GymWalk<state_size>::build_dynamics_from_response_(const nlohmann::json& response)const{
    auto dynamics = response["dynamics"];
    return dynamics;
}

template<uint_t state_size>
typename GymWalk<state_size>::time_step_type
GymWalk<state_size>::create_time_step_from_response_(const nlohmann::json& response)const{

    auto step_type   = response["time_step"]["step_type"];
    auto reward      = response["time_step"]["reward"];
    auto discount    = response["time_step"]["discount"];
    auto observation = response["time_step"]["observation"];
    auto info        = response["time_step"]["info"];
    return GymWalk::time_step_type(time_step_type_from_int(step_type),
                                                 reward, observation, discount,
                                                 std::unordered_map<std::string, std::any>());
}

template<uint_t state_size>
void
GymWalk<state_size>::make(const std::string& version,
           const std::unordered_map<std::string, std::any>& options){
			   
	if(this->is_created()){
        return;
    }
	
	auto response = api_server_.make(this->env_name(),
	                                              this->cidx(),
												  version,
												  ops);

	this->set_version_(version);
    this->make_created_();
}

template<uint_t state_size>
typename GymWalk<state_size>::time_step_type
GymWalk<state_size>::step(const action_type&  action){

#ifdef RLENVSCPP_DEBUG
     assert(this->is_created() && "Environment has not been created");
#endif

	if(this->get_current_time_step_().last()){
         return this->reset(42, std::unordered_map<std::string, std::any>());
     }

    const auto response  = api_server_.step(this -> env_name(),
											this -> cidx(),
											action);

    this->get_current_time_step_() = this->create_time_step_from_response_(response);
    return this->get_current_time_step_();
}

template<uint_t state_size>
bool
GymWalk<state_size>::is_alive()const{
	auto response = this -> api_server_.is_alive(this->env_name(), this -> cidx());
	return response["result"];

}

template<uint_t state_size>
bool
GymWalk<state_size>::close(){

	if(!this->is_created()){
        return;
    }

	auto response = this -> api_server_.close(this->env_name(), this -> cidx());
    this -> invalidate_is_created_flag_();
}

template<uint_t state_size>
typename GymWalk<state_size>::time_step_type
GymWalk<state_size>::reset(uint_t seed,
						   const std::unordered_map<std::string, std::any>& /*options*/){

    if(!this->is_created()){
#ifdef RLENVSCPP_DEBUG
     assert(this->is_created() && "Environment has not been created");
#endif
     return time_step_type();
    }
	
	auto response = this -> api_server_.reset(this->env_name(), 
	                                         this -> cidx(), seed,
											  nlohmann::json());

    this->create_time_step_from_response_(response);
    return this -> get_current_time_step_();
}

template<uint_t state_size>
GymWalk<state_size>
GymWalk<state_size>::make_copy(uint_t cidx)const{
	
	GymWalk<state_size> copy(api_server_ ,cidx);
	std::unordered_map<std::string, std::any> ops;
	auto version = this -> version();
	copy.make(version, ops);
	return copy;
}

}
}
}

#endif // GYM_WALK_H
