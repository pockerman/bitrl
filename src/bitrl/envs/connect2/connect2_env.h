/*
 * Connect2 environment from
 *  <a href="https://github.com/JoshVarty/AlphaZeroSimple">AlphaZeroSimple</a>
 * 
 * 
 * 
 * 
 */ 
#ifndef CONNECT2_ENV_H
#define CONNECT2_ENV_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/env_base.h"

#include <boost/noncopyable.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

namespace bitrl{
namespace envs{
namespace connect2{

///
/// \brief Implementation of Connect2 environment from https://github.com/JoshVarty/AlphaZeroSimple
/// Initially the environment has all its positions set to zero. When a player makes
/// a move then the position corresponding to this move 
///	
class Connect2 final: public EnvBase<TimeStep<std::vector<uint_t>>,
									 DiscreteVectorStateDiscreteActionEnv<53, 0, 4, uint_t > >
{
	
public:
	
	///
    /// \brief name
    ///
    static  const std::string name;
	
	///
	/// \brief The base type
	///
	typedef EnvBase<TimeStep<std::vector<uint_t> >,
							 DiscreteVectorStateDiscreteActionEnv<53, 0, 4, uint_t > > base_type;
							 
	
	///
	/// \brief The type of the time step
	///
    typedef typename base_type::time_step_type time_step_type;
	
	///
	/// \brief The type describing the state space for the environment
	///
	typedef typename base_type::state_space_type state_space_type;
	
	///
	/// \brief The type of the action space for the environment
	///
	typedef typename base_type::action_space_type action_space_type;

    ///
	/// \brief The type of the action to be undertaken in the environment
	///
    typedef typename base_type::action_type action_type;
	
	///
	/// \brief The state type
	///
	typedef typename base_type::state_type state_type;
	
	///
	/// \brief Expose the various reset methods we use from base class
	///
	using base_type::reset;
	
	///
    /// \brief Constructor
    ///
    Connect2();
	
	///
    /// \brief Constructor
    ///
    explicit Connect2(uint_t cidx);
	
	///
	///
	///
	Connect2(const Connect2& other);

    ///
    /// \brief make. Builds the environment. Optionally we can choose if the
    /// environment will be slippery
    ///
    virtual void make(const std::string& version,
                      const std::unordered_map<std::string, std::any>& options) override final;
					  
	///
    /// \brief step. Move in the environment with the given action
	/// This function always moves player_1
    /// \param action
    /// \return
    ///
    virtual time_step_type step(const action_type& action)override final;
	
	///
    /// \brief close
    ///
    virtual void close()override final;

	///
	/// \brief Reset the environment
	///
    virtual time_step_type reset(uint_t /*seed*/,
                                 const std::unordered_map<std::string, std::any>& /*options*/)override final;
					  
	///
	/// \brief Create a new copy of the environment with the given
	/// copy index
	///
	Connect2 make_copy(uint_t cidx)const;

    ///
    /// \brief n_states. Returns the number of states
    ///
    uint_t n_states()const noexcept{ return state_space_type::size; }

    ///
    /// \brief n_actions. Returns the number of actions
    ///
    uint_t n_actions()const noexcept{return action_space_type::size;}
	
	///
	/// \brief Make a move for the player with the given id
	///
	time_step_type move(const uint_t pid, const action_type& action);

	///
	/// \brief Returns true if the player wins
	///
	bool is_win(uint_t player)const noexcept;
	
	///
	/// \brief Returns true if there are still legal moves to do
	///
	bool has_legal_moves()const noexcept;

	///
	/// \brief Get the valid moves
	///
	std::vector<uint_t> get_valid_moves()const;

private:
	
	
	
	///
	/// \brief The discount factor
	///
	real_t discount_;
	
	///
	/// \brief The id of player 1
	///
	const uint_t player_id_1_{1};
	
	///
	/// \brief The id of player 2
	///
	const uint_t player_id_2_{2};
	
	///
	/// \brief The wining value for a player
	///
	const uint_t win_val_{2};
		
	///
	/// \brief The representation of the board
	///
	std::vector<uint_t> board_;
	
	///
	/// \brief Flag indicating if the game is finished
	///
	bool is_finished_{false};
	
};

inline
void 
Connect2::close(){
	board_ = std::vector<uint_t> ();
	this -> invalidate_is_created_flag_();
}
		
}
}
}

#endif