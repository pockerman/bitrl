#ifndef RL_SERIAL_AGENT_TRAINER_H
#define RL_SERIAL_AGENT_TRAINER_H

#include "bitrl/base/bitrl_types.h"
#include "bitsim/bitsim_consts.h"

#include "bitrl/utils/iterative_algorithm_controller.h"
#include "bitrl/utils/iterative_algorithm_result.h"

#include <boost/log/trivial.hpp>
#include <boost/noncopyable.hpp>
#include <chrono>
#include <vector>
//#include <iostream>

namespace bitrl
{
namespace rl
{

// forward declare
struct EpisodeInfo;

///
/// \brief The RLSerialTrainerConfig struct. Configuration
/// struct for the serial RL agent trainer
///
struct RLSerialTrainerConfig
{
    uint_t output_msg_frequency{bitsim::consts::INVALID_ID};
    uint_t n_episodes{0};
    real_t tolerance{bitsim::consts::TOLERANCE};
};

///
/// \detailed The RLSerialAgentTrainer class handles the training
/// for serial reinforcement learning agents
///
template <typename EnvType, typename AgentType>
class RLSerialAgentTrainer : private boost::noncopyable
{
  public:
    typedef EnvType env_type;
    typedef AgentType agent_type;

    ///
    /// \brief RLSerialAgentTrainer
    /// \param config
    /// \param agent
    ///
    RLSerialAgentTrainer(const RLSerialTrainerConfig &config, agent_type &agent);

    ///
    /// \brief train Iterate to train the agent on the given
    /// environment
    ///
    virtual bitrl::utils::IterativeAlgorithmResult train(env_type &env);

    ///
    /// \brief actions_before_training_begins.  Execute any actions
    /// the algorithm needs before starting the episode
    ///
    virtual void actions_before_training_begins(env_type &);

    ///
    /// \brief actions_before_episode_begins. Execute any actions the algorithm needs before
    /// starting the episode
    ///
    virtual void actions_before_episode_begins(env_type &, uint_t);

    ///
    /// \brief  actions_after_episode_ends. Execute any actions the algorithm needs after
    /// ending the episode
    ///
    virtual void actions_after_episode_ends(env_type &, uint_t /*episode_idx*/,
                                            const EpisodeInfo &einfo);

    ///
    /// \brief actions_after_training_ends. Execute any actions the algorithm needs after
    /// the iterations are finished
    ///
    virtual void actions_after_training_ends(env_type &);

    ///
    /// \brief episodes_total_rewards
    /// \return
    ///
    const std::vector<real_t> &episodes_total_rewards() const noexcept
    {
        return total_reward_per_episode_;
    }

    ///
    /// \brief n_itrs_per_episode
    /// \return
    ///
    const std::vector<uint_t> &n_itrs_per_episode() const noexcept { return n_itrs_per_episode_; }

  protected:
    ///
    ///
    ///
    uint_t output_msg_frequency_;

    ///
    /// \brief itr_ctrl_ Handles the iteration over the
    /// episodes
    ///
    bitrl::utils::IterativeAlgorithmController itr_ctrl_;

    ///
    /// \brief agent_
    ///
    agent_type &agent_;

    ///
    /// \brief total_reward_per_episode_
    ///
    std::vector<real_t> total_reward_per_episode_;

    ///
    /// \brief n_itrs_per_episode_ Holds the number of iterations
    /// performed per training episode
    ///
    std::vector<uint_t> n_itrs_per_episode_;
};

template <typename EnvType, typename AgentType>
RLSerialAgentTrainer<EnvType, AgentType>::RLSerialAgentTrainer(const RLSerialTrainerConfig &config,
                                                               agent_type &agent)
    : output_msg_frequency_(config.output_msg_frequency),
      itr_ctrl_(config.n_episodes, config.tolerance), agent_(agent), total_reward_per_episode_(),
      n_itrs_per_episode_()
{
}

template <typename EnvType, typename AgentType>
void RLSerialAgentTrainer<EnvType, AgentType>::actions_before_training_begins(env_type &env)
{

    agent_.actions_before_training_begins(env);
    total_reward_per_episode_.clear();
    n_itrs_per_episode_.clear();

    total_reward_per_episode_.reserve(itr_ctrl_.get_max_iterations());
    n_itrs_per_episode_.reserve(itr_ctrl_.get_max_iterations());
}

template <typename EnvType, typename AgentType>
void RLSerialAgentTrainer<EnvType, AgentType>::actions_before_episode_begins(env_type &env,
                                                                             uint_t episode_idx)
{
    agent_.actions_before_episode_begins(env, episode_idx);
}

template <typename EnvType, typename AgentType>
void RLSerialAgentTrainer<EnvType, AgentType>::actions_after_episode_ends(env_type &env,
                                                                          uint_t episode_idx,
                                                                          const EpisodeInfo &einfo)
{
    agent_.actions_after_episode_ends(env, episode_idx, einfo);
}

template <typename EnvType, typename AgentType>
void RLSerialAgentTrainer<EnvType, AgentType>::actions_after_training_ends(env_type &env)
{
    agent_.actions_after_training_ends(env);
}

template <typename EnvType, typename AgentType>
bitrl::utils::IterativeAlgorithmResult
RLSerialAgentTrainer<EnvType, AgentType>::train(env_type &env)
{

    BOOST_LOG_TRIVIAL(info) << " Start training on environment..."; //<<env.name;

    // start timing the training
    auto start = std::chrono::steady_clock::now();

    this->actions_before_training_begins(env);

    uint_t episode_counter = 0;
    bool stop_training = false;
    while (itr_ctrl_.continue_iterations())
    {

        this->actions_before_episode_begins(env, episode_counter);
        auto episode_info = agent_.on_training_episode(env, episode_counter);

        if (output_msg_frequency_ != bitsim::consts::INVALID_ID &&
            episode_counter % output_msg_frequency_ == 0)
        {

            BOOST_LOG_TRIVIAL(info) << episode_info;
        }

        total_reward_per_episode_.push_back(episode_info.episode_reward);
        n_itrs_per_episode_.push_back(episode_info.episode_iterations);
        this->actions_after_episode_ends(env, episode_counter, episode_info);

        if (episode_info.stop_training)
        {
            BOOST_LOG_TRIVIAL(info) << " Stopping training at index=" << episode_counter;

            // assume that if we were told to stop
            // that we have converge
            stop_training = true;
            break;
        }
        episode_counter += 1;
    }

    this->actions_after_training_ends(env);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<real_t> elapsed_seconds = end - start;

    BOOST_LOG_TRIVIAL(info) << " Done... ";

    auto state = itr_ctrl_.get_state();
    state.total_time = elapsed_seconds;
    state.converged = stop_training;
    return state;
}

} // namespace rl
} // namespace bitrl

#endif // RL_SERIAL_AGENT_TRAINER_H
