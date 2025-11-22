

#ifndef APISERVER_H
#define APISERVER_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/extern/HTTPRequest.hpp"
#include <string>
#include <vector>
#include <any>
#include <unordered_map>

///
/// todo write docs
///
///
///
namespace bitrl{
namespace envs{
	
///
/// \brief Utility class to facilitate
/// HTTP requests between the environments REST API 
/// and C++ drivers
///
class RESTApiServerWrapper
{
public:

    ///
    /// \brief Constructor
    ///
    explicit RESTApiServerWrapper(const std::string& url="http://0.0.0.0:8001/api",
	                              const bool initialize=true);
								  
	///
	/// \brief Returns true if the server is initialised
	///
	bool is_inisialised()const noexcept{return is_init_;}
	
	///
	/// \brief Returns the remote url
	///
	std::string get_url()const noexcept{return url_;}
	
	///
	/// \brief Return the url for the environment
	/// with the given name
	///
	std::string get_env_url(const std::string& name)const noexcept;
	
	///
	/// \brief Returns the URI of the environment
	/// with the given name Returns INVALID_STR
	/// if the environment is not registered
	///
	std::string get_uri(const std::string& name)const noexcept;
	
	///
	/// \brief Register a new environment. Throws 
	/// std::logic_error if the environment name already exists
	///
	void register_new(const std::string& name, const std::string& uri);
	
	///
	/// \brief Same as register_new but swallows the thrown exception
	///
	void register_if_not(const std::string& name, const std::string& uri);
	

	///
	/// \brief Queries the remote server if the environment with the
	/// given cidx is alive
	/// Throws std::logic_error is the environment is not registered
	///
	nlohmann::json is_alive(const std::string& env_name, 
							const uint_t cidx)const;
	
	///
	/// \brief Close the environment with the given name.
	/// Throws std::logic_error is the environment is not registered
	/// Throws std::runtime_error if the server response is not 201
	///
	nlohmann::json close(const std::string& env_name, 
	                     const uint_t cidx)const;
	
	///
	/// \brief Step in the environment with the given name
	/// and the given copy index executing action.
	/// ActionType has to be JSON serializable
	/// Throws std::logic_error is the environment is not registered
	/// Throws std::runtime_error if the server response is not 201
	///
	template<typename ActionType>
	nlohmann::json step(const std::string& env_name, 
	                    const uint_t cidx,
	                    const ActionType& action)const;
						
	///
	/// \brief Reset the the environment with the given name
	/// and the given copy index executing action.
	/// Throws std::logic_error is the environment is not registered
	/// Throws std::runtime_error if the server response is not 202
	///
	nlohmann::json reset(const std::string& env_name, 
	                     const uint_t cidx,
	                     const uint_t seed,
						 const nlohmann::json& options)const;
						 
	///
	/// \brief Make the the environment with the given name
	/// and the given copy index executing action.
	/// Throws std::logic_error is the environment is not registered
	/// Throws std::runtime_error if the server response is not 202
	///
	nlohmann::json make(const std::string& env_name, 
	                    const uint_t cidx,
	                    const std::string& version,
	                    const nlohmann::json& options)const;


	///
	/// \brief Get the dynamics of the environment. If the environment
	/// does not expose such an endpoint it returns 404
	///
	nlohmann::json dynamics(const std::string& env_name, 
							const uint_t cidx,
	                        const uint_t sidx, 
							const uint_t aidx)const;
	
	///
	/// \brief Make the cidx copy of the environment
	///
	nlohmann::json copy(const std::string& env_name,
						const uint_t cidx,
	                    const std::string& version,
	                    const nlohmann::json& options)const{return make(env_name, cidx, version, options);}

    ///
    ///
    ///
    bool has_gymnasium()const;

    ///
    ///
    ///
    std::vector<std::string> gymnasium_envs()const;

private:

	///
	/// \brief The source url of the remote server
	///
    const std::string url_;
	
	///
	/// \brief Flag indicating if the underlying
	/// environments have been initialised
	///
	bool is_init_{false};

	///
	/// \brief Map that holds the environment names and their
	/// respective URI on the remote server
	///
	std::unordered_map<std::string, std::string> envs_;

	///
	/// \brief Initialzes the available environments
	///
	void init_();
};

template<typename ActionType>
nlohmann::json 
RESTApiServerWrapper::step(const std::string& env_name, const uint_t cidx,
	                       const ActionType& action)const{
							   
		
	// find the source
	auto url_ = get_env_url(env_name);
	
	if(url_ == bitrl::consts::INVALID_STR){
		throw std::logic_error("Environment: " + env_name + " is not registered");
	}
	
	const auto request_url = url_ + "/step";
    http::Request request{request_url};
	
	nlohmann::json body;
	body["cidx"] = cidx;
	body["action"] = action;
	
	const auto response = request.send("POST", body.dump());

    if(response.status.code != 202){
        throw std::runtime_error("Environment server failed to step environment");
    }
	
	auto str_response = std::string(response.body.begin(), response.body.end());
    nlohmann::json j = nlohmann::json::parse(str_response);
	return j;
							   
}


}
}
#endif // APISERVER_H
