#ifndef MC_TREE_SEARCH_SOLVER_H
#define MC_TREE_SEARCH_SOLVER_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/mc/mcts_node.h"

#include <concepts>
#include <memory>
#include <random>
#include <utility>
#include <vector>

namespace bitrl
{
namespace rl
{
namespace algos
{
namespace mc
{

///
/// \brief What MCTSSolver needs from an environment: it is copied for every
/// simulation, has a fixed number of discrete actions, and step() returns a
/// time step with the reward and whether the episode ended
///
template <typename EnvTp>
concept mcts_env_concept = std::copy_constructible<EnvTp> &&
    requires(EnvTp env, typename EnvTp::action_type a)
{
    typename EnvTp::state_type;
    {
        env.n_actions()
        } -> std::convertible_to<uint_t>;
    {
        env.step(a).reward()
        } -> std::convertible_to<real_t>;
    {
        env.step(a).done()
        } -> std::convertible_to<bool>;
};

///
/// \brief The default rollout policy of MCTSSolver: every action with the same
/// probability
///
struct UniformRolloutPolicy
{
    template <typename EnvTp>
    typename EnvTp::action_type operator()(const EnvTp &env, std::mt19937 &generator) const
    {
        std::uniform_int_distribution<uint_t> action(0, static_cast<uint_t>(env.n_actions()) - 1);
        return static_cast<typename EnvTp::action_type>(action(generator));
    }
};

///
/// \brief The configuration of MCTSSolver
///
struct MCTSConfig
{
    ///
    /// \brief How many simulations plan() runs before it picks an action
    ///
    uint_t n_simulations{200};

    ///
    /// \brief The most steps a simulation looks ahead, in the tree and in the
    /// rollout after it
    ///
    uint_t max_depth{20};

    ///
    /// \brief How much UCB favours children that were visited less. It has the
    /// units of the returns, so scale it with them
    ///
    real_t temperature{1.0};

    ///
    /// \brief The discount factor of the returns
    ///
    real_t gamma{1.0};

    ///
    /// \brief The seed of the generator of the random rollouts and of the order
    /// the children are tried in
    ///
    uint_t seed{42};
};

///
/// \brief Monte Carlo tree search, with UCB to pick the children (UCT).
///
/// MCTS plans from the state the environment is in. plan(env) grows a tree of
/// action sequences from that state. Every simulation copies the environment
/// and
/// - walks down the tree, always to the child with the largest upper
///   confidence bound, until it reaches a node with a child it has not tried;
/// - adds that child, by taking its action;
/// - goes on with the actions of the rollout policy until the episode ends or
///   the simulation is max_depth steps deep;
/// - adds to the score of every node on its path the reward of the step into
///   the node and the discounted return that followed it.
/// plan() then returns the action of the child of the root that was visited
/// most. The environment the simulations run in can be a model of the world
/// the agent acts in, as long as it is fast to copy and to step.
///
/// The rollout policy is called as policy(env, generator), with the
/// environment of the simulation and the solver's random generator, and
/// returns an action. Random actions, the default, need no knowledge of the
/// problem; a policy that plays sensibly gives returns that say much more
/// about the node the rollout starts from.
///
template <mcts_env_concept EnvTp, typename RolloutPolicy = UniformRolloutPolicy> class MCTSSolver
{
  public:
    typedef EnvTp env_type;
    typedef typename env_type::action_type action_type;
    typedef typename env_type::state_type state_type;
    typedef MCTSNodeBase<action_type, state_type> node_type;

    explicit MCTSSolver(const MCTSConfig &config, RolloutPolicy rollout_policy = RolloutPolicy())
        : config_(config), rollout_policy_(std::move(rollout_policy)), generator_(config.seed)
    {
    }

    ///
    /// \brief plan. Runs config().n_simulations simulations from copies of env
    /// and returns the action that is best from the state env is in. env is
    /// not changed
    ///
    action_type plan(const env_type &env);

    ///
    /// \brief The root of the tree of the last plan(), or null before the first
    ///
    std::shared_ptr<const node_type> root() const noexcept { return root_; }

    const MCTSConfig &config() const noexcept { return config_; }

  private:
    MCTSConfig config_;
    RolloutPolicy rollout_policy_;
    std::mt19937 generator_;
    std::shared_ptr<node_type> root_;

    void simulate_(const env_type &env);
    void expand_(const std::shared_ptr<node_type> &node, uint_t n_actions);
};

template <mcts_env_concept EnvTp, typename RolloutPolicy>
typename MCTSSolver<EnvTp, RolloutPolicy>::action_type
MCTSSolver<EnvTp, RolloutPolicy>::plan(const env_type &env)
{
    root_ = std::make_shared<node_type>(nullptr, action_type{});
    expand_(root_, static_cast<uint_t>(env.n_actions()));

    for (uint_t i = 0; i < config_.n_simulations; ++i)
    {
        simulate_(env);
    }

    // the most visited child is the most reliable choice; ties go to the
    // larger mean return
    std::shared_ptr<node_type> best;
    for (uint_t c = 0; c < root_->n_children(); ++c)
    {
        auto child = root_->get_child(c);
        if (!best || child->total_visits() > best->total_visits() ||
            (child->total_visits() == best->total_visits() && child->total_visits() > 0 &&
             child->win_pct() > best->win_pct()))
        {
            best = child;
        }
    }
    return best->get_action();
}

template <mcts_env_concept EnvTp, typename RolloutPolicy>
void MCTSSolver<EnvTp, RolloutPolicy>::expand_(const std::shared_ptr<node_type> &node,
                                               uint_t n_actions)
{
    for (uint_t a = 0; a < n_actions; ++a)
    {
        node->add_child(node, static_cast<action_type>(a));
    }
    // try the children in a random order, so that ties do not always go to
    // the first action
    node->shuffle_children(generator_);
}

template <mcts_env_concept EnvTp, typename RolloutPolicy>
void MCTSSolver<EnvTp, RolloutPolicy>::simulate_(const env_type &env)
{
    env_type sim(env);
    const auto n_actions = static_cast<uint_t>(sim.n_actions());

    // the nodes the simulation goes through, from the root, and the reward
    // of the step into each of them
    std::vector<std::shared_ptr<node_type>> path{root_};
    std::vector<real_t> rewards;
    bool done = false;

    // Selection: down the tree while every child of the node was tried
    auto node = root_;
    while (!done && path.size() <= config_.max_depth && node->has_children() &&
           node->n_explored_children() == node->n_children())
    {
        node = node->max_ucb_child(config_.temperature);
        auto time_step = sim.step(node->get_action());
        rewards.push_back(time_step.reward());
        done = time_step.done();
        path.push_back(node);
    }

    // Expansion: try a child the node has not tried yet
    if (!done && path.size() <= config_.max_depth)
    {
        if (!node->has_children())
        {
            expand_(node, n_actions);
        }
        auto child = node->get_child(node->n_explored_children());
        node->update_explored_children();
        auto time_step = sim.step(child->get_action());
        rewards.push_back(time_step.reward());
        done = time_step.done();
        path.push_back(child);
    }

    // Rollout: the rollout policy to the end of the episode or of the look ahead
    real_t rollout_return = 0.0;
    real_t discount = 1.0;
    for (auto depth = path.size() - 1; !done && depth < config_.max_depth; ++depth)
    {
        auto time_step = sim.step(rollout_policy_(sim, generator_));
        rollout_return += discount * time_step.reward();
        discount *= config_.gamma;
        done = time_step.done();
    }

    // Backpropagation: every node scores the reward of the step into it and
    // the discounted return that followed, which is what its parent compares
    // its children by. After the last node comes the rollout
    real_t g = rollout_return;
    for (auto i = path.size(); i-- > 1;)
    {
        g = rewards[i - 1] + config_.gamma * g;
        path[i]->update_visits();
        path[i]->update_total_score(g);
    }
    // the root scores the whole return of the simulation
    root_->update_visits();
    root_->update_total_score(g);
}

} // namespace mc
} // namespace algos
} // namespace rl
} // namespace bitrl

#endif // MC_TREE_SEARCH_SOLVER_H
