#ifndef ENV_BASE_H
#define ENV_BASE_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"

#include <unordered_map>
#include <any>
#include <string>
#include <type_traits>


namespace bitrl{
namespace envs{

/**
	 * @brief Base class interface for Reinforcement Learning environments.
	 *
	 * This class defines the minimum API contract that any RL environment
	 * must implement. It exposes functionality for creating, resetting,
	 * and stepping through an environment, while tracking configuration
	 * such as version, options, and current state.
	 *
	 * @tparam TimeStepType Type returned after each environment step
	 * @tparam SpaceType Environment's space interface type providing
	 *                   state and action space definitions
	 */
template<typename TimeStepType, typename SpaceType>
class EnvBase: public SpaceType
{
public:
	
	static_assert(std::is_default_constructible<TimeStepType>::value &&
		          "TimeStepType should be default constructible");
	static_assert(std::is_default_constructible<SpaceType>::value &&
		          "SpaceType should be default constructible");
	
	/** @brief Default seed used in reset() if none provided */
	static const uint_t DEFAULT_ENV_SEED = 42;

	/** @brief Alias for the type returned when stepping the environment */
    typedef TimeStepType time_step_type;
	
	/** @brief Type describing the environment state space */
	typedef typename SpaceType::state_space state_space_type;
	
	/** @brief Type describing an individual state */
	typedef typename SpaceType::state_type state_type;
	
	/** @brief Type describing the environment action space */
	typedef typename SpaceType::action_space action_space_type;

	/** @brief Type representing an individual action */
    typedef typename SpaceType::action_type action_type;

    /** @brief Virtual destructor */
    virtual ~EnvBase()=default;

	/**
	  * @brief Construct the environment instance.
	  *
	  * @param version  Version string used to control environment variant
	  * @param make_options  Key-value configuration options for environment creation.
	  * @param reset_options Key-value configuration how the environment should be reset
	  *
	  * @note Derived classes should use set_version_() and set_make_options_()
	  *       internally. They may store selected options for later use.
	  */
    virtual void make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& make_options,
                      const std::unordered_map<std::string, std::any>& reset_options) = 0;

	/** @brief Close and release any acquired environment resources */
    virtual void close()=0;
	
	/**
	 * @brief Reset the environment to an initial state using the reset
	 * options specified during make.
	 *
	 * @return Initial time step after reset
	 */
    virtual time_step_type reset()=0;

	/**
	 * @brief Perform one step in the environment using an action.
	 *
	 * @param action Action applied to the environment
	 * @return New time step after executing the action
	 */
    virtual time_step_type step(const action_type& action)=0;

	/**
	 * @brief Access the configuration options provided to make().
	 * @return Map of option keys and values
	 */
	const std::unordered_map<std::string, std::any>& make_options()const noexcept{return make_options_;}

	/**
	 * @brief Access the configuration options provided to make().
	 * @return Map of option keys and values
	 */
	const std::unordered_map<std::string, std::any>& reset_options()const noexcept{return reset_options_;}

	/**
	 * @brief Read a specific make() option and cast it to the requested type.
	 *
	 * @tparam T Expected data type
	 * @param op_name Key of the option to read
	 * @return Requested value if present
	 * @throws std::bad_any_cast If stored type does not match T
	 */
	template<typename T>
	T read_option(const std::string& op_name)const;

	/**
	 * @brief Get the id identifying this environment within a simulation batch.
	 * The id is valid only if make has been called
	 * @return Copy index
	 */
    std::string idx()const noexcept{return idx_;}

	/**
	* @brief Check if make() has successfully initialized the environment.
	* @return True if environment is ready, false otherwise
	*/
	bool is_created()const noexcept{return is_created_;}

	/**
	  * @brief Get the name of this environment instance.
	  * @return Environment name
	 */
	std::string env_name()const noexcept{return name_;}

	/**
	 * @brief Get the environment version set during make().
	 * @return Version string
	 */
	std::string version()const noexcept{return version_;}

protected:

	/**
	 * @brief Constructor (protected — for subclassing only).
	 * @param cidx Copy index used in multi-environment simulations
	 * @param name Name of the environment instance
	 */
    explicit EnvBase(const std::string& idx=bitrl::consts::INVALID_STR,
	                 const std::string& name=bitrl::consts::INVALID_STR);
					 
	/** @brief Copy constructor */
	EnvBase(const EnvBase&);

	/**
	 * @brief Set internal version string.
	 * @note Should be called only inside make()
	 */
    void set_version_(const std::string& version )noexcept{version_ = version;}

	/**
	 * @brief Set the id of the environment
	 * @param idx
	 */
	void set_idx_(const std::string& idx)noexcept{idx_ = idx;}
	
	/** @brief Store make() options for future access */
	void set_make_options_(const std::unordered_map<std::string, std::any>& options) noexcept{make_options_ = options;}

	/** @brief Mark environment as not created */
    void invalidate_is_created_flag_()noexcept{is_created_ = false;}

	/** @brief Mark environment creation as successful */
    void make_created_()noexcept{is_created_= true;}

	/** @brief Mutable access to the current time step */
	time_step_type& get_current_time_step_()noexcept{return current_state_;}

	/** @brief Read-only access to the current time step */
    const time_step_type& get_current_time_step_()const noexcept{return current_state_;}
	
private:

    bool is_created_;  ///< Indicates that make() has finished successfully
    std::string idx_; ///< Environment instance id
    std::string version_; ///< Environment version identifier
    const std::string name_; ///< Environment name
	std::unordered_map<std::string, std::any> make_options_; ///< Copied options from make()
	std::unordered_map<std::string, std::any> reset_options_; ///< Copied options from make()
    time_step_type current_state_; ///< Latest environment time step
};


template<typename TimeStepType, typename SpaceType>
EnvBase<TimeStepType, SpaceType>::EnvBase(const std::string& idx, const std::string& name)
:
SpaceType(),
is_created_(false),
idx_(idx),
version_(),
name_(name),
current_state_()
{}

template<typename TimeStepType, typename SpaceType>
EnvBase<TimeStepType, SpaceType>::EnvBase(const EnvBase<TimeStepType, SpaceType>& other)
:
SpaceType(),
is_created_(other.is_created_),
idx_(other.idx_),
version_(other.version_),
name_(other.name_),
current_state_()
{}

template<typename TimeStepType, typename SpaceType>
void
EnvBase<TimeStepType, SpaceType>::close(){
	this -> is_created_ = false;
}

template<typename TimeStepType, typename SpaceType>
template<typename T>
T 
EnvBase<TimeStepType, SpaceType>::read_option(const std::string& op_name)const{
	
	auto op_itr = make_options_.find(op_name);
	if(op_itr != make_options_.end()){
		return std::any_cast<T>(op_itr -> second);
	}
	
	throw std::logic_error("Option: " + op_name + " not found");
}

template<typename TimeStepType, typename SpaceType>
void EnvBase<TimeStepType, SpaceType>::make(const std::string& version,
						  const std::unordered_map<std::string, std::any>& make_options,
						  const std::unordered_map<std::string, std::any>& reset_options)
{
	version_ = version;
	make_options_ = make_options;
	reset_options_ = reset_options;
}

}
}

#endif // ENV_BASE_H
