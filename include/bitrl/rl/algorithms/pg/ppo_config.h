#ifndef PPO_CONFIG_H
#define PPO_CONFIG_H


#include "bitrl/base/bitrl_types.h"
//#include "rlenvs/rlenvs_consts.h"
#include "bitrl/utils/train_enum_type.h"

#include <ostream>
#include <string>

namespace bitrl {
namespace rl {
namespace algos {
namespace pg {
	
//using namespace rlenvscpp::consts;
	
///
/// \brief The PPOConfig struct. Configuration for PPOSolver class
///
struct PPOConfig
{

    ///
    /// \brief Discount factor
    ///
    real_t gamma{0.99};

	///
	/// \brief The epsilon factor to use
	///
    real_t epsilon{0.01};
	
	///
	/// \brief Flag indicating whether to clip the policy grad
	///
 	bool clip_policy_grad{false};
	
	///
	/// \brief Flag indicating whether to clip the critic grad
	///
 	bool clip_critic_grad{false};

    ///
    /// \brief The value to clip the gradient for the policy
    ///
    real_t max_grad_norm_policy{1.0};
	
	///
	/// \brief The value to clip the gradient for the actor
	///
	real_t max_grad_norm_critic{1.0};
	
	///
    /// \brief Number of training episodes
    ///
    uint_t n_episodes{100};

    ///
    /// \brief Number of iterations per episode
    ///
    uint_t max_itrs_per_episode{100};

    ///
    /// \brief How large the experince buffer should be
    ///
    uint_t buffer_size{100};

	uint_t max_passes_over_batch{4};


	real_t clip_epsilon {0.5};

    ///
    ///
    ///
    bool normalize_advantages{true};

    ///
    ///
    ///
    DeviceType device_type{DeviceType::CPU};

    ///
    ///
    ///
    std::string save_model_path{""};
	
	///
    /// \brief print
    /// \param out
    /// \return
    ///
    std::ostream& print(std::ostream& out)const;
	
	///
	/// \brief Load the configuration from the given json file
	///
	void load_from_json(const std::string& filename);
};

	
inline
std::ostream& operator<<(std::ostream& out, const PPOConfig& opts){
    return opts.print(out);
}

} // pg
} // algos
} // rl
} // bitrl

#endif