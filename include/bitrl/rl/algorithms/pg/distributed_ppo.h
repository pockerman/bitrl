#ifndef DISTRIBUTED_PPO_H
#define DISTRIBUTED_PPO_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/pg/parallel_ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/distributed/communicator.h"
#include "bitrl/rl/distributed/torch_parameters.h"
#include "bitrl/rl/episode_info.h"

#include <torch/torch.h>

#include <memory>
#include <vector>

namespace bitrl
{
namespace rl::algos::pg
{

///
/// \brief Proximal policy optimisation over several ranks, threads, processes or machines,
/// that average their gradients.
///
/// Every rank has its own copies of the environment, a bitrl::envs::ParallelEnvs seeded
/// differently on every rank, and its own copy of the networks and their optimisers. Every
/// training round each rank plays an episode in each of its copies, as ParallelPPOSolver does.
/// Then, at every pass over the batch, each rank computes the gradients of its own episodes, the
/// ranks replace them with their mean over all the ranks, through a
/// bitrl::rl::distributed::Communicator, and every rank's optimisers take the same step.
///
/// Before training, every rank takes the networks of rank 0, so the ranks start alike. The mean
/// of the gradients is the same on every rank to the last bit, so the ranks stay alike: the
/// networks of the ranks are one network, trained on the episodes of all of them. This is the
/// data parallelism of PyTorch's DistributedDataParallel.
///
/// Each rank normalises the advantages of its own episodes and averages the loss over its own
/// steps, so every rank counts the same in the mean, whatever the length of its episodes.
///
/// The EpisodeInfo of a round gives the mean return of the episodes of all the ranks, and its
/// iterations the steps of all of them.
///
/// Every rank must train for the same number of rounds. The policy and the critic are as for
/// ParallelPPOSolver; their parameters must be float32.
///
template <typename EnvType, typename PolicyType, typename CriticType>
class DistributedPPOSolver final : public ParallelPPOSolver<EnvType, PolicyType, CriticType>
{
  public:
    typedef ParallelPPOSolver<EnvType, PolicyType, CriticType> base_type;
    typedef typename base_type::env_type env_type;
    typedef PolicyType policy_type;
    typedef CriticType critic_type;

    ///
    /// \param communicator how the ranks meet; it must outlive the solver
    ///
    DistributedPPOSolver(const PPOConfig &config, policy_type &policy, critic_type &critic,
                         std::unique_ptr<torch::optim::Optimizer> &policy_optimizer,
                         std::unique_ptr<torch::optim::Optimizer> &critic_optimizer,
                         rl::distributed::Communicator &communicator)
        : base_type(config, policy, critic, policy_optimizer, critic_optimizer),
          communicator_(communicator)
    {
    }

    ///
    /// \brief Take the networks of rank 0, then get ready to train
    ///
    void actions_before_training_begins(env_type &envs) override
    {
        base_type::actions_before_training_begins(envs);
        rl::distributed::broadcast_parameters(communicator_, parameters_(), 0);
    }

    EpisodeInfo on_training_episode(env_type &envs, uint_t round) override
    {
        auto info = base_type::on_training_episode(envs, round);
        // the returns and the steps of all the ranks
        std::vector<double> totals{0.0, static_cast<double>(this->last_returns().size()),
                                   static_cast<double>(info.episode_iterations)};
        for (const real_t r : this->last_returns())
        {
            totals[0] += r;
        }
        communicator_.all_reduce_sum(totals);
        info.episode_reward = totals[0] / totals[1];
        info.episode_iterations = static_cast<uint_t>(totals[2]);
        global_returns_.push_back(info.episode_reward);
        return info;
    }

    ///
    /// \brief The mean return of the episodes of all the ranks, per round
    ///
    const std::vector<real_t> &global_returns() const noexcept { return global_returns_; }

    ///
    /// \brief A checksum of the networks, the same on every rank
    ///
    std::uint64_t checksum() const
    {
        return rl::distributed::parameters_checksum(
            const_cast<DistributedPPOSolver *>(this)->parameters_());
    }

    const rl::distributed::Communicator &communicator() const noexcept { return communicator_; }

  protected:
    void on_gradients_() override
    {
        rl::distributed::average_gradients(communicator_, parameters_());
    }

  private:
    std::vector<torch::Tensor> parameters_()
    {
        auto p = this->policy_->parameters();
        const auto c = this->critic_->parameters();
        p.insert(p.end(), c.begin(), c.end());
        return p;
    }

    rl::distributed::Communicator &communicator_;
    std::vector<real_t> global_returns_;
};

} // namespace rl::algos::pg
} // namespace bitrl

#endif // USE_PYTORCH
#endif // DISTRIBUTED_PPO_H
