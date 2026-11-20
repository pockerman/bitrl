#ifndef A2C_CONFIG_H
#define A2C_CONFIG_H

#include "bitrl/base/bitrl_types.h"
//#include "bitsim/rlenvs_consts.h"
#include "bitrl/utils/train_enum_type.h"

#include <ostream>
#include <string>

namespace bitrl {
namespace rl {
namespace algos {
namespace pg {
	
//using namespace rlenvscpp::consts;
	
///
/// \brief The A2CConfig struct. Configuration for A2CSolver class
///
struct A2CConfig
{

    ///
    /// \brief Discount factor
    ///
    real_t gamma{0.99};

    ///
    /// \brief GAE lambda
    ///
    real_t lambda{0.1};

    ///
    /// \brief Coefficient for accounting for entropy contribution
    ///
    real_t beta{0.0};

    ///
    /// \brief policy_loss_weight. How much weight to give
    /// on the policy loss when forming the global loss
    ///
    real_t policy_loss_weight{ 1.0};

    ///
    /// 
    ///
    real_t value_loss_weight{1.0};
	
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
    ///
    ///
    uint_t buffer_size{100};

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
std::ostream& operator<<(std::ostream& out, const A2CConfig& opts){
    return opts.print(out);
}

}
}
}
}

#endif