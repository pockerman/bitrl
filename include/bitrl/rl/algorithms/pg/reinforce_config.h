#ifndef REINFORCE_CONFIG_H
#define REINFORCE_CONFIG_H

#include "bitrl/base/bitrl_types.h"
#include "bitsim/bitsim_consts.h"
#include "bitrl/utils/train_enum_type.h"

#include <ostream>
#include <string>

namespace bitrl {
namespace rl {
namespace algos {
namespace pg {

///
/// \brief Enumeration of the baseline types supported
///
enum class BaselineEnumType { NONE=-1, CONSTANT=0, MEAN=1, STANDARDIZE=2};
	

///
/// \struct ReinforceConfig configuration class for REINFORCE algorithm
/// \brief The ReinforceOpts struct. Holds various
/// configuration options for the Reinforce algorithm
///
struct ReinforceConfig
{
	bool normalize_rewards{false};
    
	///
	/// \brief How to train the algorithm
	///
	bitrl::utils::TrainEnumType train_type{bitrl::utils::TrainEnumType::BATCH};
	
	///
	/// \brief The baseline to use
	///
	BaselineEnumType baseline_type{BaselineEnumType::NONE};
	
	///
	/// \brief The device type that PyTorch calculations take place
	///
	DeviceType device_type;
	
	///
	/// \brief The number of episodes
	///
	uint_t n_episodes;
 
	///
	/// \brief Max number of iterations per
	/// episode
	///
	uint_t max_itrs_per_episode;
	
	///
	/// \brief The discount factor
	///
    real_t gamma;
	
	///
	/// \brief The constant to use when baseline_type = BaselineEnumType::CONSTANT
	///
	real_t baseline_constant{0.0};
	
	///
	/// \brief Small constant to use as tolerance
	/// Used when baseline_type = BaselineEnumType::STANDARDIZE 
	///
	real_t eps{bitsim::consts::TOLERANCE};
    

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
std::ostream& operator<<(std::ostream& out, ReinforceConfig opts){
    return opts.print(out);
}

}
}
}
}

#endif