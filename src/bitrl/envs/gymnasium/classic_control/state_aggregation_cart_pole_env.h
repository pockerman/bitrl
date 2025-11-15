#ifndef STATE_AGGREGATION_CART_POLE_ENV_H
#define STATE_AGGREGATION_CART_POLE_ENV_H

/**
  *
  * TiledCartPole environment. Wrapper to CartPole environment using state aggregation. The original environment
  *  is described here: https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py
  *  The state variables are:
  *
  * Observation:
  *      Type: Box(4)
  *      Num     Observation               Min                     Max
  *      0       Cart Position             -2.4                    2.4
  *      1       Cart Velocity             -Inf                    Inf
  *      2       Pole Angle                -0.209 rad (-12 deg)    0.209 rad (12 deg)
  *      3       Pole Angular Velocity     -Inf                    Inf
  *  Actions:
  *      Type: Discrete(2)
  *      Num   Action
  *      0     Push cart to the left
  *      1     Push cart to the right
  *      Note: The amount the velocity that is reduced or increased is not
  *      fixed; it depends on the angle the pole is pointing. This is because
  *      the center of gravity of the pole increases the amount of energy needed
  *      to move the cart underneath it
  *  Reward:
  *      Reward is 1 for every step taken, including the termination step
  *  Starting State:
  *      All observations are assigned a uniform random value in [-0.05..0.05]
  *  Episode Termination:
  *      Pole Angle is more than 12 degrees.
  *      Cart Position is more than 2.4 (center of the cart reaches the edge of
  *      the display).
  *      Episode length is greater than 200.
  *      Solved Requirements:
  *      Considered solved when the average return is greater than or equal to
  *      195.0 over 100 consecutive trials.
  **/

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/envs/gymnasium/classic_control/cart_pole_env.h"
#include "bitrl/envs/state_aggregation_env_base.h"

#include <string>
#include <utility>
#include <vector>
#include <tuple>

namespace bitrl{

/// Forward declaration
template<typename StateTp> class TimeStep;


namespace envs{
namespace gymnasium{

/*
///
/// \brief The TiledCartPoleBoundaries struct. Boundaries for
/// the state variables in the CartPole environment
///
struct StateAggregationCartPoleBoundaries
{

    std::pair<real_t,real_t> cart_pole_pos{-2.4, 2.4};
    std::pair<real_t,real_t> cart_pole_vel{-4.0, 4.0};
    std::pair<real_t,real_t> cart_pole_theta{-0.20943951, 0.20943951};
    std::pair<real_t,real_t> cart_pole_theta_vel{-4.0, 4.0};
};

///
/// \brief The TiledCartPole class
///
class StateAggregationCartPole final: public StateAggregationEnvBase<CartPole, std::tuple<uint_t, uint_t, uint_t, uint_t>>
{
public:

    typedef StateAggregationEnvBase<CartPole, std::tuple<uint_t, uint_t, uint_t, uint_t>>::time_step_type time_step_type;
    typedef StateAggregationEnvBase<CartPole, std::tuple<uint_t, uint_t, uint_t, uint_t>>::state_type state_type;
    typedef StateAggregationEnvBase<CartPole, std::tuple<uint_t, uint_t, uint_t, uint_t>>::action_type action_type;

    ///
    /// \brief Constructor
    ///
    StateAggregationCartPole(const std::string version, obj_t main_namespace, uint_t n_states,
                  uint_t state_idx=4, const StateAggregationCartPoleBoundaries& boundaries=StateAggregationCartPoleBoundaries());

    ///
    /// \brief reset
    /// \return
    ///
    time_step_type reset();

    ///
    /// \brief make
    ///
    virtual void make() override final;

    ///
    /// \brief step
    /// \param action
    /// \return
    ///
    virtual time_step_type step(const action_type& action) override final;

    ///
    /// \brief create_bins. Create the bins for the variables
    ///
    virtual void create_bins() override final;

    ///
    /// \brief get_state Transforms the original state to the aggregate state
    /// \param obs. The original continuous state
    /// \return
    ///
    virtual state_type get_state(const CartPole::state_type& obs)const override final;

    ///
    /// \brief n_actions
    /// \return
    ///
    uint_t n_actions()const noexcept{return this->env_.n_actions();}

    ///
    /// \brief get_states
    /// \return
    ///
    const std::vector<std::tuple<uint_t, uint_t, uint_t, uint_t>>& get_states()const noexcept{return discrete_observation_space_;}

    ///
    /// \brief get_states
    /// \return
    ///
    std::vector<std::tuple<uint_t, uint_t, uint_t, uint_t>> get_states() noexcept{return discrete_observation_space_;}

private:

    ///
    /// \brief state_idx_
    ///
    uint_t state_idx_;

    ///
    /// \brief boundaries_
    ///
    const StateAggregationCartPoleBoundaries boundaries_;

    ///
    /// \brief bins for the pole position space
    ///
    std::vector<real_t> pole_theta_space_;

    ///
    /// \brief bins for the pole velocity space
    ///
    std::vector<real_t> pole_theta_velocity_space_;

    ///
    /// \brief bins for the cart position space
    ///
    std::vector<real_t> cart_pos_space_;

    ///
    /// \brief bins for the cart velocity space
    ///
    std::vector<real_t> cart_vel_space_;

    ///
    /// \brief discrete_observation_space_
    ///
    std::vector<std::tuple<uint_t, uint_t, uint_t, uint_t> > discrete_observation_space_;

    ///
    /// \brief current_state_
    ///
    time_step_type current_state_;

    void build_pole_theta_space_();
    void build_cart_position_space_();
    void build_cart_velocity_space_();
    void build_pole_theta_velocity_space_();
    void create_states_();
};
*/

}
}
}

#endif // TILED_CART_POLE_ENV_H
