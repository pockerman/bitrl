#include "rest_rl_env_client.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/extern/HTTPRequest.hpp"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>
#include <stdexcept>
#include <iostream>

namespace bitrl{
namespace network{

RESTRLEnvClient::RESTRLEnvClient(const std::string& url, const bool initialize)
:
url_(url),
is_init_(false),
envs_()
{
  if(initialize){
	  init_();
  }
}

void 
RESTRLEnvClient::init_(){
	
	envs_["FrozenLake"] = "/gymnasium/frozen-lake-env";
	envs_["Taxi"] = "/gymnasium/taxi-env";
	envs_["CliffWalking"] = "/gymnasium/cliff-walking-env";
	envs_["BlackJack"] = "/gymnasium/black-jack-env";
	envs_["MountainCar"] = "/gymnasium/mountain-car-env";
	envs_["CartPole"] = "/gymnasium/cart-pole-env";
	envs_["Acrobot"] = "/gymnasium/acrobot-env";
	envs_["Pendulum"] = "/gymnasium/pendulum-env";
	envs_["AcrobotV"] = "/gymnasium/acrobot-env/v";
	envs_["GymWalk"]= "/gdrl/gym-walk-env";
	
}


bool
RESTRLEnvClient::is_env_registered(const std::string& env_name)const noexcept
{
	auto env_itr = envs_.find(env_name);
	if( env_itr != envs_.end() )
	{
		return true;
	}
	return false;
}

void 
RESTRLEnvClient::register_new(const std::string& name, const std::string& uri){
	
	auto env_itr = envs_.find(name);
	if(env_itr == envs_.end()){
		envs_[name] = uri;
		return;
	}
	
	throw std::logic_error("Environment: " + name + " already exists");
}

void 
RESTRLEnvClient::register_if_not(const std::string& name, const std::string& uri){
	
	try{
		register_new(name, uri);
	}
	catch(const std::logic_error& e){}
}

std::string 
RESTRLEnvClient::get_uri(const std::string& name)const noexcept{
	
	auto env_itr = envs_.find(name);
	if(env_itr == envs_.end()){
		return bitrl::consts::INVALID_STR;
	}
	
	return env_itr -> second;
}

std::string 
RESTRLEnvClient::get_env_url(const std::string& name)const noexcept{
	
	auto uri_ = get_uri(name);
	if(uri_ == bitrl::consts::INVALID_STR){
		return bitrl::consts::INVALID_STR;
	}
	
	return get_url() + uri_;
}

nlohmann::json 
RESTRLEnvClient::is_alive(const std::string& env_name, const std::string& idx)const{
	
	// find the source
	auto url_ = get_env_url(env_name);
	
	if(url_ == bitrl::consts::INVALID_STR){
		throw std::logic_error("Environment: " + env_name + " is not registered");
	}

    http::Request request{url_ + "/" + idx + "/is-alive"};
    const auto response = request.send("GET");
    
	auto str_response = std::string(response.body.begin(), response.body.end());
    nlohmann::json j = nlohmann::json::parse(str_response);
	return j;	

}

nlohmann::json 
RESTRLEnvClient::close(const std::string& env_name, const std::string& idx)const{
	
	// find the source
	auto url_ = get_env_url(env_name);
	
	if(url_ == bitrl::consts::INVALID_STR){
		throw std::logic_error("Environment: " + env_name + " is not registered");
	}
	
    http::Request request{url_ + "/" + idx + "/close"};
    const auto response = request.send("POST");

	if(response.status.code != 202){
        throw std::runtime_error("Could not close environment " + env_name);
    }
	
	auto str_response = std::string(response.body.begin(), response.body.end());
    nlohmann::json j = nlohmann::json::parse(str_response);
	return j;	
}

nlohmann::json 
RESTRLEnvClient::reset(const std::string& env_name, const std::string& idx,
                            const uint_t seed, const nlohmann::json& options)const{
								
	
    // find the source
	auto url_ = get_env_url(env_name);
	
	if(url_ == bitrl::consts::INVALID_STR){
		throw std::logic_error("Environment: " + env_name + " is not registered");
	}

	const auto request_url = url_ + "/" + idx + "/reset";
    http::Request request{request_url};

    nlohmann::json request_body;
    request_body["seed"] = seed;
	request_body["options"] = options;


	std::cout<<"Sending body request: "<<request_body.dump()<<std::endl;
    const auto response = request.send("POST", request_body.dump());

     if(response.status.code != 202){
        throw std::runtime_error("Environment server failed to reset environment");
    }
								
	auto str_response = std::string(response.body.begin(), response.body.end());
    nlohmann::json j = nlohmann::json::parse(str_response);
	return j;							
								
}


nlohmann::json 
RESTRLEnvClient::make(const std::string& env_name,
					  const std::string& version,
					  const nlohmann::json& options)const{

	// find the source
	auto url_ = get_env_url(env_name);
	
	if(url_ == bitrl::consts::INVALID_STR){
		throw std::logic_error("Environment: " + env_name + " is not registered");
	}
	
	const auto request_url = url_ + "/make";
    http::Request request{request_url};
	
    nlohmann::json request_body;
    request_body["version"] = version;
	request_body["options"] = options;

	std::cout<<"Sending body request: "<<request_body.dump()<<std::endl;
	
    const auto response = request.send("POST", request_body.dump());

    if(response.status.code != 201){
        throw std::runtime_error("Environment server failed to create Environment");
    }
	
	auto str_response = std::string(response.body.begin(), response.body.end());
    nlohmann::json j = nlohmann::json::parse(str_response);
	return j;	
								
}

nlohmann::json 
RESTRLEnvClient::dynamics(const std::string& env_name, const std::string& idx,
                          const uint_t sidx, const uint_t aidx)const{
								   
	// find the source
	auto url_ = get_env_url(env_name);
	
	if(url_ == bitrl::consts::INVALID_STR){
		throw std::logic_error("Environment: " + env_name + " is not registered");
	}

	auto request_url = url_ + "/" + idx + "/dynamics?state_id="+std::to_string(sidx);
	if (aidx != consts::INVALID_ID)
	{
		request_url += "&action_id="+std::to_string(aidx);
	}

	http::Request request{request_url};
	const auto response = request.send("GET");

	auto str_response = std::string(response.body.begin(), response.body.end());
	nlohmann::json j = nlohmann::json::parse(str_response);
	return j;

}

bool
RESTRLEnvClient::has_gymnasium()const{

    const auto request_url = url_ + "/api-info/gymnasium";
    http::Request request{request_url};

    const auto response = request.send("GET");
    return response.status.code == 200;

}

std::vector<std::string>
RESTRLEnvClient::gymnasium_envs()const{

    const auto request_url = url_ + "/api-info/gymnasium/envs";
    http::Request request{request_url};

    const auto response = request.send("GET");

    if(response.status.code != 200){
        throw std::runtime_error("Environment server responded with error");
    }

    auto str_response = std::string(response.body.begin(), response.body.end());
    using json = nlohmann::json;
    json j = json::parse(str_response);
    return j["envs"];
}

}
}
