#ifndef POLICY_ITERATION_H
#define POLICY_ITERATION_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/dp/dp_algo_base.h"
#include "bitrl/rl/algorithms/dp/iterative_policy_evaluation.h"
#include "bitrl/rl/algorithms/dp/policy_improvement.h"

#include "bitsim//utils/io/csv_file_writer.h"
#include "bitsim/bitsim_consts.h"

#include <string>

namespace bitrl{
namespace rl::algos::dp
{


    ///
/// \brief The PolicyIterationConfig struct
///
    struct PolicyIterationConfig
    {
        uint_t n_policy_eval_steps;
        real_t gamma{1.0};
        real_t tolerance{1.0e-6};
        std::string save_path{bitsim::consts::INVALID_STR};
    };

    ///
/// \brief The policy iteration class
///
    template<typename EnvType, typename PolicyType>
    class PolicyIterationSolver final: public DPSolverBase<EnvType>
    {
    public:

        ///
    /// \brief env_t
    ///
        typedef typename DPSolverBase<EnvType>::env_type env_type;

        ///
    /// \brief policy_type
    ///
        typedef PolicyType policy_type;

        ///
    /// \brief PolicyIteration
    ///
        PolicyIterationSolver(PolicyIterationConfig config,
                              uint_t action_space_size,
                              policy_type& policy);

        ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
        virtual void actions_before_training_begins(env_type& env)override;

        ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
        virtual void actions_after_training_ends(env_type& /*env*/)override;

        ///
    /// \brief actions_before_training_episode
    ///
        virtual void actions_before_episode_begins(env_type&, uint_t /*episode_idx*/)override{}

        ///
    /// \brief actions_after_training_episode
    ///
        virtual void actions_after_episode_ends(env_type&, uint_t /*episode_idx*/,
                                                const EpisodeInfo& /*einfo*/)override{}

        ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
        virtual EpisodeInfo on_training_episode(env_type& env, uint_t episode_idx) override;

        ///
    /// \brief save
    /// \param filename
    ///
        void save(const std::string& filename)const;

    private:

        PolicyIterationConfig config_;

        ///
    /// \brief v_ The value function vector
    ///
        DynVec<real_t> v_;

        ///
    /// \brief policy_eval_
    ///
        IterativePolicyEvalutationSolver<env_type, policy_type> policy_eval_;


        ///
    /// \brief policy_imp_
    ///
        PolicyImprovement<env_type, policy_type> policy_impr_;

    };

    template<typename EnvType, typename PolicyType>
    PolicyIterationSolver<EnvType, PolicyType>::PolicyIterationSolver(PolicyIterationConfig config,
                                                                      uint_t action_space_size,
                                                                      policy_type& policy)
        :
        DPSolverBase<EnvType>(),
        config_(config),
        v_(),
        policy_eval_({config.gamma, config.tolerance}, policy),
        policy_impr_(action_space_size, config.gamma, DynVec<real_t>(), policy)
    {}


    template<typename EnvType, typename PolicyType>
    void
    PolicyIterationSolver<EnvType, PolicyType>::actions_before_training_begins(env_type& env){

        policy_eval_.actions_before_training_begins(env);
        policy_impr_.actions_before_training_begins(env);
    }

    template<typename EnvType, typename PolicyType>
    void
    PolicyIterationSolver<EnvType, PolicyType>::actions_after_training_ends(env_type& /*env*/){
        v_ = policy_eval_.get_value_function();

        if(config_.save_path != bitsim::consts::INVALID_STR){
            save(config_.save_path);
        }
    }

    template<typename EnvType, typename PolicyType>
    EpisodeInfo
    PolicyIterationSolver<EnvType, PolicyType>::on_training_episode(env_type& env, uint_t episode_idx){

        auto start = std::chrono::steady_clock::now();
        EpisodeInfo info;

        auto episode_rewards = 0.0;

        // make a copy of the policy already obtained
        auto old_policy = policy_eval_.get_policy();

        for(uint_t itr=0; itr < config_.n_policy_eval_steps; ++itr ){
            // evaluate the policy
            policy_eval_.on_training_episode(env, itr);
        }

        // update the value function to
        // improve for
        policy_impr_.set_value_function( policy_eval_.get_value_function());

        // improve the policy
        auto policy_imp_info = policy_impr_.on_training_episode(env, episode_idx);

        // get the improved policy
        const auto& new_policy = policy_impr_.policy();

        // policy converged
        if(old_policy == new_policy){
            info.stop_training = true;
        }

        policy_eval_.update_policy(new_policy);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<real_t> elapsed_seconds = end-start;


        info.episode_index = episode_idx;
        info.episode_reward = episode_rewards;
        info.episode_iterations = config_.n_policy_eval_steps + policy_imp_info.episode_iterations;
        info.total_time = elapsed_seconds;
        return info;
    }

    template<typename EnvType, typename PolicyType>
    void
    PolicyIterationSolver<EnvType, PolicyType>::save(const std::string& filename)const{

        bitsim::utils::io::CSVWriter file_writer(filename, ',');
        file_writer.open();

        file_writer.write_column_names({"state_index", "value_function"});

        auto vec_size = static_cast<uint_t>(v_.size());
        for(uint_t s=0; s < vec_size; ++s){
            auto row = std::make_tuple(s, v_[s]);
            file_writer.write_row(row);
        }
    }

}
}


#endif // POLICY_ITERATION_H
