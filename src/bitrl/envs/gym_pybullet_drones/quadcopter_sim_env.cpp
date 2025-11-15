// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#include "rlenvs/envs/gym_pybullet_drones/quadcopter_sim_env.h"
#include "rlenvs/rlenvscpp_config.h"
#include "rlenvs/extern/HTTPRequest.hpp"
#include "rlenvs/extern/nlohmann/json/json.hpp"


#ifdef RLENVSCPP_DEBUG
#include <cassert>
#endif


#include <iostream>

namespace rlenvscpp{
namespace envs{
namespace gym_pybullet_drones{


void
QuadcopterSimEnv::make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& /*options*/){

    if(this->is_created()){
        return;
    }

    const auto request_url = this->get_url_str() + "/make";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;
    j["version"] = version;
    auto body = j.dump();
    const auto response = request.send("POST", body);

    if(response.status.code != 201){
        throw std::runtime_error("Environment server failed to create Environment");
    }


    this->make_created_();
    this->set_version_(version);
}


QuadcopterSimEnv::time_step_type
QuadcopterSimEnv::step(const action_type& action){


    if(!this->is_created()){
#ifdef RLENVSCPP_DEBUG
     assert(this->is_created() && "Environment has not been created");
#endif
     return time_step_type();
    }

    const auto request_url = this->get_url_str() + "/step";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;
    j["action"] = action;
    auto body = j.dump();
    const auto response = request.send("POST", body);

     if(response.status.code != 202){
        throw std::runtime_error("Environment server failed to reset environment");
    }

    current_state_ = this->create_time_step_from_response_(response);
    return current_state_;


}


QuadcopterSimEnv::time_step_type
QuadcopterSimEnv::reset(uint_t seed,
                        const std::unordered_map<std::string, std::any>& /*options*/){



    if(!this->is_created()){
#ifdef RLENVSCPP_DEBUG
     assert(this->is_created() && "Environment has not been created");
#endif
     return time_step_type();
    }

    const auto request_url = this->get_url_str() + "/reset";
    http::Request request{request_url};


    using json = nlohmann::json;
    json j;
    j["seed"] = seed;
    auto body = j.dump();
    const auto response = request.send("POST", body);

     if(response.status.code != 202){
        throw std::runtime_error("Environment server failed to reset environment");
    }

    current_state_ = this->create_time_step_from_response_(response);
    return current_state_;

}


bool
QuadcopterSimEnv::is_alive()const noexcept{

    http::Request request{this->get_url_str() + "/is-alive"};
    const auto response = request.send("GET");
    const auto str_response = std::string(response.body.begin(), response.body.end());
    auto pos = str_response.find("true");

    if (pos == std::string::npos){
        return false;
    }

    return true;
}

void
QuadcopterSimEnv::close(){

     if(!this->is_created()){
        return;
    }

    auto url = this->get_url_str();
    http::Request request{url + "/close"};
    const auto response = request.send("POST");
    this->invalidate_is_created_flag_();


}


QuadcopterSimEnv::time_step_type
QuadcopterSimEnv::create_time_step_from_response_(const http::Response& response)const{

    auto str_response = std::string(response.body.begin(), response.body.end());

    std::cout<<"String response: "<<str_response<<std::endl;
    using json = nlohmann::json;

    json j = json::parse(str_response);

    auto step_type = j["time_step"]["step_type"];
    auto reward = j["time_step"]["reward"];
    auto discount = j["time_step"]["discount"];
    auto observation = j["time_step"]["observation"];
    auto info = j["time_step"]["info"];
    return QuadcopterSimEnv::time_step_type(time_step_type_from_int(step_type),
                                            reward, observation, discount,
                                            std::unordered_map<std::string, std::any>());


}

}
}
}
