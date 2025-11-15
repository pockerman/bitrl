#ifndef WITH_REST_API_MIXIN_H
#define WITH_REST_API_MIXIN_H

#include "rlenvs/extern/HTTPRequest.hpp"
#include <string>

namespace rlenvscpp{
namespace envs{
	
	
///
/// \brief class synchronized_env_mixin
///
template<typename TimeStepType>
class with_rest_api_mixin
{
public:
	
	typedef TimeStepType time_step_type;
	
	///
	/// \brief Constructor 
	///
	with_rest_api_mixin(const std::string& api_url, const std::string& resource_path);
	
	///
	/// \brief Copy ctor
	///
	with_rest_api_mixin(const with_rest_api_mixin& other);
	
	///
	/// \brief destructor
	///
	virtual ~with_rest_api_mixin()=default;
	
	///
    /// \brief build the time step from the server response
    ///
    virtual time_step_type create_time_step_from_response_(const http::Response& response)const=0;
	
	///
    /// \brief Query the environment server is the environment has been created
    ///
    virtual bool is_alive()const=0;
	
	///
	/// \brief Returns the url the environment is using
	///
    std::string get_url()const noexcept{return api_url_ + resource_path_;}

    ///
	/// \brief Returns the url the environment is using
	///
    std::string get_api_url()const noexcept{return api_url_;}
	
	///
	/// \brief The resource path
	///
	std::string get_resource_path()const noexcept {return resource_path_;}
	

private:
	
	std::string api_url_;
	std::string resource_path_;
	
		
};

template<typename TimeStepType>
with_rest_api_mixin<TimeStepType>::with_rest_api_mixin(const std::string& api_url, const std::string& resource_path)
:
api_url_(api_url),
resource_path_(resource_path)
{}

template<typename TimeStepType>
with_rest_api_mixin<TimeStepType>::with_rest_api_mixin(const with_rest_api_mixin<TimeStepType>& other)
:
api_url_(other.api_url_),
resource_path_(other.resource_path_)
{}
	
}
}

#endif 