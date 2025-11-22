/*
 * Quadcopter simulation environment
 * from 
 * <a href="https://github.com/utiasDSL/gym-pybullet-drones/tree/main">gym-pybullet-drones</a>
 * 
 * 
 * 
 * 
 */ 

#ifndef QUADCOPTER_SIM_ENV_H
#define QUADCOPTER_SIM_ENV_H

#include "rlenvs/rlenvs_types_v2.h"
#include "rlenvs/envs/env_base.h"
#include "rlenvs/time_step.h"
#include "rlenvs/extern/HTTPRequest.hpp"

#include <boost/noncopyable.hpp>

#include <unordered_map>
#include <any>
#include <string>

namespace rlenvscpp{
namespace envs{
namespace gym_pybullet_drones{

class QuadcopterSimEnv: public EnvBase<TimeStep<std::vector<real_t>>, std::vector<real_t>>
{
public:


    typedef typename EnvBase<TimeStep<std::vector<real_t>>, std::vector<real_t>>::time_step_type time_step_type;
    typedef typename EnvBase<TimeStep<std::vector<real_t>>, std::vector<real_t>>::action_type action_type;

    /**
     * @brief constructor
     *
     */
    QuadcopterSimEnv(const std::string& url, const std::string& name);


    /**
      * \brief make. Builds the environment. Optionally we can choose if the
      * environment will be slippery
      */
    virtual void make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& options) override final;


    /**
      * \brief step
      * \param action
      * \return
      */
    virtual time_step_type step(const action_type& action) override final;


     /**
     * @brief close the environment
     */
    virtual void close() override final;


    /**
      * @brief Reset the environment
      */
    virtual time_step_type reset(uint_t seed,
                                 const std::unordered_map<std::string, std::any>& options) override final;


   /**
      * \brief Query the environment server is the environment has been created
      */
    bool is_alive()const noexcept;


private:


    time_step_type current_state_;


    /**
      * \brief build the time step from the server response
      */
    time_step_type create_time_step_from_response_(const http::Response& response)const;


};


inline
QuadcopterSimEnv::QuadcopterSimEnv(const std::string& url, const std::string& name)
:
EnvBase(url + "/gym-pybullet-drones/quadcopter-sim-env", name),
current_state_()
{}


}
}
}

#endif // QUADCOPTER_SIM_ENV_H
