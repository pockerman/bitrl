

#ifndef REST_RL_ENV_CLIENT_H
#define REST_RL_ENV_CLIENT_H

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/HTTPRequest.hpp"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <any>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>


namespace bitrl
{
namespace network
{

///
/// \brief Utility class to facilitate
/// HTTP requests between the environments REST API
/// and C++ drivers
///
class RESTRLEnvClient
{
  public:
    ///
    /// \brief Constructor
    ///
    explicit RESTRLEnvClient(const std::string &url = "http://0.0.0.0:8001/api",
                             const bool initialize = true);

    ///
    /// \brief Returns true if the server is initialised
    ///
    bool is_inisialised() const noexcept { return is_init_; }

    ///
    /// \brief Returns the remote url
    ///
    std::string get_url() const noexcept { return url_; }

    /**
     * Returns true if the environment with the given name is registered
     * @param env_name The environment name to query
     * @return
     */
    bool is_env_registered(const std::string &env_name) const noexcept;

    ///
    /// \brief Return the url for the environment
    /// with the given name
    ///
    std::string get_env_url(const std::string &name) const noexcept;

    ///
    /// \brief Returns the URI of the environment
    /// with the given name Returns INVALID_STR
    /// if the environment is not registered
    ///
    std::string get_uri(const std::string &name) const noexcept;

    ///
    /// \brief Register a new environment. Throws
    /// std::logic_error if the environment name already exists
    ///
    void register_new(const std::string &name, const std::string &uri);

    ///
    /// \brief Same as register_new but swallows the thrown exception
    ///
    void register_if_not(const std::string &name, const std::string &uri);

    /**
     * Return the number of copies of the environment with the given name
     * @param name
     * @return
     */
    nlohmann::json n_copies(const std::string &name, const std::string& version=bitrl::consts::INVALID_STR) const;

    ///
    /// \brief Queries the remote server if the environment with the
    /// given cidx is alive
    /// Throws std::logic_error is the environment is not registered
    ///
    nlohmann::json is_alive(const std::string &env_name, const std::string &idx) const;

    ///
    /// \brief Close the environment with the given name.
    /// Throws std::logic_error is the environment is not registered
    /// Throws std::runtime_error if the server response is not 201
    ///
    nlohmann::json close(const std::string &env_name, const std::string &idx) const;

    ///
    /// \brief Step in the environment with the given name
    /// and the given copy index executing action.
    /// ActionType has to be JSON serializable
    /// Throws std::logic_error is the environment is not registered
    /// Throws std::runtime_error if the server response is not 201
    ///
    template <typename ActionType>
    nlohmann::json step(const std::string &env_name, const std::string &idx,
                        const ActionType &action) const;

    ///
    /// \brief Reset the the environment with the given name
    /// and the given copy index executing action.
    /// Throws std::logic_error is the environment is not registered
    /// Throws std::runtime_error if the server response is not 202
    ///
    nlohmann::json reset(const std::string &env_name, const std::string &idx, const uint_t seed,
                         const nlohmann::json &options) const;

    ///
    /// \brief Make the the environment with the given name
    /// and the given copy index executing action.
    /// Throws std::logic_error is the environment is not registered
    /// Throws std::runtime_error if the server response is not 202
    ///
    nlohmann::json make(const std::string &env_name, const std::string &version,
                        const nlohmann::json &options) const;

    ///
    /// \brief Get the dynamics of the environment. If the environment
    /// does not expose such an endpoint it returns 404
    ///
    nlohmann::json dynamics(const std::string &env_name, const std::string &idx, const uint_t sidx,
                            const uint_t aidx) const;

    bool has_gymnasium() const;
    std::vector<std::string> gymnasium_envs() const;

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

template <typename ActionType>
nlohmann::json RESTRLEnvClient::step(const std::string &env_name, const std::string &idx,
                                     const ActionType &action) const
{

    // find the source
    auto url_ = get_env_url(env_name);

    if (url_ == bitrl::consts::INVALID_STR)
    {
        throw std::logic_error("Environment: " + env_name + " is not registered");
    }

    const auto request_url = url_ + "/" + idx + "/step";
    http::Request request{request_url};

    nlohmann::json body;
    body["action"] = action;

    std::cout << "Sending body: " << body.dump() << std::endl;

    const auto response = request.send("POST", body.dump());
    if (response.status.code != 202)
    {
        throw std::runtime_error("Environment server failed to step environment");
    }

    auto str_response = std::string(response.body.begin(), response.body.end());
    nlohmann::json j = nlohmann::json::parse(str_response);
    return j;
}

} // namespace network
} // namespace bitrl
#endif // APISERVER_H
