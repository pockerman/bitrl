#ifndef MC_TREE_SEARCH_BASE_H
#define MC_TREE_SEARCH_BASE_H

#include "cubeai/base/cubeai_types.h"
#include "cubeai/rl/algorithms/rl_algorithm_base.h"
#include "cubeai/rl/algorithms/rl_algo_config.h"
#include "cubeai/utils/iteration_mixin.h"
#include "cubeai/rl/episode_info.h"
#include "cubeai/rl/algorithms/mc/mcts_node.h"

#include "cubeai/base/cubeai_config.h"

#ifdef CUBEAI_DEBUG
#include <cassert>
#endif

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace cubeai{
namespace rl{
namespace algos{
namespace mc{


///
/// \brief The MCTreeSearchConfig struct
///
struct MCTreeSearchConfig: public RLAlgoConfig
{
    uint_t max_tree_depth = 1000;
    real_t temperature = 1.0;

};





///
/// \brief MCTreeSearchBase.
///
template<typename EnvTp, typename NodeTp=MCTSNodeBase<typename EnvTp::action_type, typename EnvTp::state_type>>
class MCTSSolver final: public RLSolverBase<EnvTp>
{
public:

    static_assert (std::is_default_constructible<typename EnvTp::action_type>::value, 
	               "Action type is not default constructible");
    
	static_assert (std::is_default_constructible<typename EnvTp::state_type>::value, 
	               "State type is not default constructible");

    ///
    /// \brief env_type
    ///
    typedef EnvTp env_type;

    ///
    /// \brief node_type
    ///
    typedef NodeTp node_type;
	
	///
	/// \brief The time step type
	///
	typedef typename env_type::time_step_type time_step_type;
	
	///
    /// \brief MCTreeSearchBase
    /// \param config
    ///
    MCTSSolver(MCTreeSearchConfig config);
	
	///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
    virtual void actions_before_training_begins(env_type&)override final{}
	
	///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
    virtual void actions_after_training_ends(env_type&) override final{}
	
	///
    /// \brief actions_before_training_episode
    ///
    virtual void actions_before_episode_begins(env_type&, uint_t /*episode_idx*/){}

    ///
    /// \brief actions_after_training_episode
    ///
    virtual void actions_after_episode_ends(env_type&, uint_t /*episode_idx*/, const EpisodeInfo& /*einfo*/){}

    ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
    virtual EpisodeInfo on_training_episode(env_type&, uint_t /*episode_idx*/) override final;

    ///
    /// \brief simulate_node
    /// \param node
    ///
    time_step_type simulate_node(std::shared_ptr<node_type> node, env_type& env);

    ///
    /// \brief expand_node
    /// \param node
    ///
    void expand_node(std::shared_ptr<node_type> node, env_type& env);

    ///
    /// \brief backprop
    ///
    void backprop(std::shared_ptr<node_type> node);

    ///
    /// \brief max_depth_tree
    /// \return
    ///
    uint_t max_depth_tree()const noexcept{return max_depth_tree_;}

protected:

    ///
    /// \brief itr_mix_
    ///
    IterationMixin itr_mix_;

    ///
    /// \brief root_
    ///
    std::shared_ptr<node_type> root_;

    ///
    /// \brief max_depth_tree_
    ///
    uint_t max_depth_tree_;

    ///
    /// \brief temperature_
    ///
    real_t temperature_;

};

template<typename EnvTp, typename NodeTp>
MCTSSolver<EnvTp, NodeTp>::MCTSSolver(MCTreeSearchConfig config)
    :
      RLSolverBase(config.n_episodes, config.tolerance),
      itr_mix_(config.n_itrs_per_episode, config.render_episode),
      root_(nullptr),
      max_depth_tree_(config.max_tree_depth),
      temperature_(config.temperature)
{}


template<typename EnvTp, typename NodeTp>
EpisodeInfo
MCTSSolver<EnvTp, NodeTp>::on_training_episode(env_type& env, uint_t /*episode_idx*/){

    auto best_reward = std::numeric_limits<real_t>::min();

	// on every episode we reset
	// we may want to reset the copy here
	auto time_step = env.reset();

    while(this->itr_mix_.continue_iterations()){

        sum_reward_ = 0.0;
        actions_.clear();

        auto node = this->root_;

        auto time_step = this->simulate_node(node, env_copy);

        auto terminal = time_step.done();

        if(!terminal){
            // expand the node if this is not
            // terminal
            this->expand_node(node, env_copy);
        }

        // creating exhaustive list of actions
        while(!terminal){
           auto action = env_copy.sample_action();
           auto new_time_step = env_copy.step(action);
           sum_reward_ += new_time_step.reward();
           actions_.push_back(action);

           if(actions_.size() > this->max_depth_tree()){
               sum_reward_ -= 100;
               break;
           }
        }

        // do some book keeping retaining the best reward value and actions
        if( best_reward < sum_reward_){
            best_reward = sum_reward_;
            best_actions_ = actions_;
        }

        // backpropagating in MCTS for assigning reward value to a node.
        this->backprop(node);
    }

    // step in the best actions
    sum_reward_ = 0.;
    for(auto action : best_actions_){
        auto time_step = this->env.step(action);

         sum_reward_ += time_step.reward();
         if(time_step.done()){
             break;
         }
    }


    best_rewards_.push_back(sum_reward_);
}
}

template<typename EnvTp, typename NodeTp>
void 
MCTSSolver<EnvTp, NodeTp>::backprop(std::shared_ptr<node_type> node){
	
	while(node){

        node -> update_visits();
        node -> update_total_score(sum_reward_);
        node = node ->parent();
    }
	
}

template<typename Env, typename NodeTp>
void
MCTSSolver<Env>::expand_node(std::shared_ptr<node_type> node, env_type& env){

    for(uint_t a=0; a < env.n_actions(); ++a ){
        node -> add_child(node, a);
    }

    node->shuffle_children();

}


template<typename Env, typename NodeTp>
typename  MCTSSolver<Env, NodeTp>::time_step_type
MCTSSolver<Env, NodeTp>::simulate_node(std::shared_ptr<node_type> node, env_type& env){

    auto time_step = MCTSSolver<Env, NodeTp>::time_step_type();
    while(node -> has_children()){

        if(node -> n_explored_children() < node -> n_children() ){

            auto child = node->get_child(node->n_explored_children());
            node ->update_explored_children();
            node = child;
        }
        else{

            node = node -> max_ucb_child(this->temperature_);

        }

        time_step  = env.step(node ->get_action());
        sum_reward_ += time_step.reward();
        actions_.push_back(node ->get_action());
    }

    return time_step;
}


}
}
}
}

#endif // MC_TREE_SEARCH_BASE_H
