#ifndef BLACK_JACK_H
#define BLACK_JACK_H

/**
 *  BlackJack environment
 *  https://github.com/Farama-Foundation/Gymnasium/blob/main/gymnasium/envs/toy_text/blackjack.py
 */

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/toy_text_base.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/network/rest_rl_env_client.h"

#include <any>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace bitrl
{
namespace envs::gymnasium
{

/**
 * BlackJack class. Wrapper to the Blackjack OpenAI-Gym environment.
 *
 * This environment is part of the Toy Text environments which contains general information about
 * the environment. Action Space: Discrete(2) Observation Space: Tuple(Discrete(32), Discrete(11),
 * Discrete(2)) Blackjack is a card game where the goal is to beat the dealer by obtaining cards
 * that sum to closer to 21 (without going over 21) than the dealers cards. The game starts with the
 * dealer having one face up and one face down card, while the player has two face up cards. All
 * cards are drawn from an infinite deck (i.e. with replacement). The card values are:
 * - Face cards (Jack, Queen, King) have a point value of 10.
 * - Aces can either count as 11 (called a ‘usable ace’) or 1.
 * - Numerical cards (2-10) have a value equal to their number.
 * The player has the sum of cards held.
 * The player can request additional cards (hit) until they decide to stop (stick) or exceed 21
 * (bust, immediate loss). After the player sticks, the dealer reveals their facedown card, and
 * draws cards until their sum is 17 or greater. If the dealer goes bust, the player wins. If
 * neither the player nor the dealer busts, the outcome (win, lose, draw) is decided by whose sum is
 * closer to 21. This environment corresponds to the version of the blackjack problem described in
 * Example 5.1 in Reinforcement Learning: An Introduction by Sutton and Barto [1].
 *
 * Action Space
 * The action shape is (1,) in the range {0, 1} indicating whether to stick or hit.
 * 0: Stick
 * 1: Hit
 * Observation Space
 * The observation consists of a 3-tuple containing: the player’s current sum,
 * the value of the dealer’s one showing card (1-10 where 1 is ace), and whether the player holds a
 * usable ace (0 or 1). The observation is returned as (int(), int(), int()). Rewards win game: +1
 * lose game: -1
 * draw game: 0
 * win game with natural blackjack: +1.5 (if natural is True) +1 (if natural is False)
 * Episode End
 * The episode ends if the following happens:
 * Termination:
 * The player hits and the sum of hand exceeds 21.
 * The player sticks.
 * An ace will always be counted as usable (11) unless it busts the player.
 *
 */
class BlackJack final : public ToyTextEnvBase<TimeStep<std::vector<uint_t>>, 48, 2>
{

  public:
    ///
    /// \brief name
    ///
    static const std::string name;

    ///
    /// \brief The URI for accessing the environment
    ///
    static const std::string URI;

    ///
    /// \brief The base type
    ///
    typedef typename ToyTextEnvBase<TimeStep<std::vector<uint_t>>, 48, 2>::base_type base_type;

    ///
    /// \brief The time step type we return every time a step in the
    /// environment is performed
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
    /// \brief The type of the action to be undertaken in the environment
    ///
    typedef typename base_type::state_type state_type;

    BlackJack(network::RESTRLEnvClient &api_server);

    BlackJack(const BlackJack &other);

    ~BlackJack() override = default;

    ///
    /// \brief make. Builds the environment. Optionally we can choose if the
    /// environment will be slippery
    ///
    virtual void
    make(const std::string &version, const std::unordered_map<std::string, std::any> &options,
         const std::unordered_map<std::string, std::any> &reset_options) override final;

    ///
    /// \brief step
    /// \param action
    /// \return
    ///
    virtual time_step_type step(const action_type &action) override final;
    bool is_natural() const noexcept { return is_natural_; }
    bool is_sab() const noexcept { return is_sab_; }

  protected:
    ///
    /// \brief build the dynamics from response
    ///
    virtual dynamics_t build_dynamics_from_response_(const nlohmann::json &) const override final;

    ///
    /// \brief Handle the reset response from the environment server
    ///
    virtual time_step_type
    create_time_step_from_response_(const nlohmann::json &) const override final;

  private:
    ///
    /// \brief Flag indicating if the environment has been
    /// initialized as natural
    ///
    bool is_natural_{false};
    bool is_sab_{false};

    void make_natural() noexcept { is_natural_ = true; }
    void make_sab() noexcept { is_sab_ = true; }
};

} // namespace envs::gymnasium
} // namespace bitrl
#endif // BLACK_JACK_H
