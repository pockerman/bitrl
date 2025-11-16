#ifndef ENV_BASE_H
#define ENV_BASE_H

///
/// \file env_base.h
///
#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/envs/synchronized_env_mixin.h"

#include <unordered_map>
#include <any>
#include <string>
#include <type_traits>


namespace bitrl{
namespace envs{


///
/// \class EnvBase
/// \brief Base class for environments.
///
/// The EnvBase class establishes the minimum contract
/// that an RL environment should expose.
///
template<typename TimeStepType, typename SpaceType>
class EnvBase: public SpaceType, public synchronized_env_mixin
{
public:
	
	static_assert(std::is_default_constructible<TimeStepType>::value && "TimeStepType should be default constructible");
	static_assert(std::is_default_constructible<SpaceType>::value && "SpaceType should be default constructible");
	
	///
	/// \brief Default seed to use
	///
	static const uint_t DEFAULT_ENV_SEED = 42;

	///
	/// \brief The time step type we return every time a step in the
	/// environment is performed
	///
    typedef TimeStepType time_step_type;
	
	///
	/// \brief The type describing the state space for the environment
	///
	typedef typename SpaceType::state_space state_space_type;
	
	///
	/// \brief The type of the state
	///
	typedef typename SpaceType::state_type state_type;
	
	///
	/// \brief The type of the action space for the environment
	///
	typedef typename SpaceType::action_space action_space_type;

    ///
	/// \brief The type of the action to be undertaken in the environment
	///
    typedef typename SpaceType::action_type action_type;

    ///
	/// \brief Destructor 
	///
    virtual ~EnvBase()=default;

    ///
	/// \brief make. Builds the environment.
    /// \param version. the version of the environment to build
	/// \param options. Options to use for building the environment.
	/// Concrete classes may choose to hold a copy 
	///
    virtual void make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& options) = 0;

    ///
	/// \brief close the environment
	///
    virtual void close()=0;
	
	/// 
	/// \brief Reset the environment
	/// \param seed. The seed to use for resetting the environment
	/// \param options. Options to use for resetting the environment.
	///
    virtual time_step_type reset(uint_t seed,
                                 const std::unordered_map<std::string, std::any>& options)=0;
								 
	///
	/// \brief Reset the environment always using the same seed
	///
    time_step_type reset(){
        return reset(DEFAULT_ENV_SEED, std::unordered_map<std::string, std::any>());}
		
	///
	/// \brief Reset the environment always using the provided seed
	///
    time_step_type reset(uint_t seed){
        return reset(seed, std::unordered_map<std::string, std::any>());}
								 
	///
	/// \brief step in the environment by performing the given action
    /// \param action. The action to execute in the environment 
	/// \return An instance of time_step_type 
    virtual time_step_type step(const action_type& action)=0;
	
    ///
	/// \brief is_created Returns true is make has been called successfully
	///
    bool is_created()const noexcept{return is_created_;}

    ///
	/// \brief Returns the version of the environment
	///
    std::string version()const noexcept{return version_;}

    ///
	/// \brief Returns the name of the environment
	///
    std::string env_name()const noexcept{return name_;}
	
	///
	/// \brief Returns a read reference to the options passed when calling make
	///
	const std::unordered_map<std::string, std::any>& make_options()const noexcept{return make_options_;}
	
	///
	/// \brief Read the option with the given name
	///
	template<typename T>
	T read_option(const std::string& op_name)const;

    ///
	/// \brief Returns the index of the environment that is active within
	/// a simulation 
	///
    uint_t cidx()const noexcept{return cidx_;}

protected:

    ///
	/// \brief Constructor
	///
    explicit EnvBase(const uint_t cidx=0, 
	                 const std::string& name=bitrl::consts::INVALID_STR);
					 
	///
	/// \brief Copy constructor
	///
	EnvBase(const EnvBase&);

    ///
	/// \brief Helper function to set the version. 
	/// To be called only when the make is called
	///
    void set_version_(const std::string& version )noexcept{version_ = version;}
	
	///
	/// \brief Set the make options
	///
	void set_make_options_(const std::unordered_map<std::string, std::any>& options) noexcept{make_options_ = options;}

	///
	/// \brief 
	///
    void invalidate_is_created_flag_()noexcept{is_created_ = false;}

    ///
	/// \brief mark the environment as created
	///
    void make_created_()noexcept{is_created_= true;}
	
	time_step_type& get_current_time_step_()noexcept{return current_state_;}
    const time_step_type& get_current_time_step_()const noexcept{return current_state_;}
	
private:

	///
	/// \brief Flag indicating if the environment has been created
	///
    bool is_created_;
	
	///
	/// The copy idx
	///
    uint_t cidx_;
	
	///
	/// \brief Version of the environment
	///
    std::string version_;
	
	///
	/// \brief Name of the environment
	///
    const std::string name_;
	
	///
	/// \brief Copy of the options upon calling make
	///
	std::unordered_map<std::string, std::any> make_options_;
	
	///
    /// \brief current_state
    ///
    time_step_type current_state_;
};


template<typename TimeStepType, typename SpaceType>
EnvBase<TimeStepType, SpaceType>::EnvBase(const uint_t cidx, const std::string& name)
:
SpaceType(),
synchronized_env_mixin(),
is_created_(false),
cidx_(cidx),
version_(),
name_(name),
current_state_()
{}

template<typename TimeStepType, typename SpaceType>
EnvBase<TimeStepType, SpaceType>::EnvBase(const EnvBase<TimeStepType, SpaceType>& other)
:
SpaceType(),
synchronized_env_mixin(),
is_created_(other.is_created_),
cidx_(other.cidx_),
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

}
}

#endif // ENV_BASE_H
