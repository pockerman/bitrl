#ifndef Q_LEARNING_H
#define Q_LEARNING_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/episode_info.h"
#include "bitrl/rl/algorithms/td/td_algo_base.h"
#include "bitrl/rl/worlds/envs_concepts.h"
#include "bitrl/rl/policies/max_tabular_policy.h"
#include "bitrl/maths/matrix_utilities.h"

#include "bitsim/utils/io/csv_file_writer.h"
#include "bitsim/bitsim_consts.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <chrono>

namespace bitrl {
namespace rl::algos::td
{

	///
/// \brief The QLearningConfig struct
///
	struct QLearningConfig
	{
		bool average_episode_reward{true};
		uint_t n_episodes;
		uint_t max_num_iterations_per_episode;
		real_t tolerance;
		real_t gamma;
		real_t eta;
		std::string path{bitsim::consts::INVALID_STR};

	};


	///
/// \brief The QLearning class. Table based implementation
/// of the Q-learning algorithm using epsilon-greedy policy.
/// The implementation also allows for exponential decay
/// of the used epsilon
///
	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	class QLearningSolver final: public TDAlgoBase<EnvTp>
	{

	public:

		///
    /// \brief env_t
    ///
		typedef typename TDAlgoBase<EnvTp>::env_type env_type;

		///
    /// \brief action_t
    ///
		typedef typename TDAlgoBase<EnvTp>::action_type action_type;

		///
    /// \brief state_t
    ///
		typedef typename TDAlgoBase<EnvTp>::state_type state_type;

		///
    /// \brief action_selector_t
    ///
		typedef PolicyType policy_type;

		///
    /// \brief Constructor
    ///
		QLearningSolver(const QLearningConfig config, const PolicyType& policy);

		///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
		virtual void actions_before_training_begins(env_type&);

		///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
		virtual void actions_after_training_ends(env_type&);

		///
    /// \brief actions_before_training_episode
    ///
		virtual void actions_before_episode_begins(env_type&, uint_t /*episode_idx*/){}

		///
    /// \brief actions_after_training_episode
    ///
		virtual void actions_after_episode_ends(env_type&, uint_t episode_idx,
		                                        const EpisodeInfo& /*einfo*/);

		///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
		virtual EpisodeInfo on_training_episode(env_type&, uint_t episode_idx);

		///
    /// \brief Save the state-action function in a CSV format
    ///
		void save(const std::string& filename)const;

		///
	/// \brief Build the policy after training
	///
		bitrl::rl::policies::MaxTabularPolicy build_policy()const;

	private:

		///
    /// \brief config_
    ///
		QLearningConfig config_;

		///
    /// \brief action_selector_
    ///
		policy_type policy_;

		///
    /// \brief q_table_. The tabilar representation of the Q-function
    ///
		DynMat<real_t> q_table_;

		///
    /// \brief update_q_table_
    /// \param action
    ///
		void update_q_table_(const action_type& action, const state_type& cstate,
		                     const state_type& next_state, const  action_type& next_action,
		                     real_t reward);

	};

	template <envs::discrete_world_concept EnvTp, typename PolicyType>
	QLearningSolver<EnvTp, PolicyType>::QLearningSolver(const QLearningConfig config,
	                                                    const PolicyType& policy)
		:
		TDAlgoBase<EnvTp>(),
		config_(config),
		policy_(policy),
		q_table_()
	{}

	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	void
	QLearningSolver<EnvTp, PolicyType>::actions_before_training_begins(env_type& env){
		q_table_ = DynMat<real_t>(env.n_states(), env.n_actions());

		for(uint_t i=0; i < env.n_states(); ++i)
			for(uint_t j=0; j < env.n_actions(); ++j)
				q_table_(i, j) = 0.0;

	}

	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	void
	QLearningSolver<EnvTp, PolicyType>::actions_after_training_ends(env_type&){

		if(config_.path != bitsim::consts::INVALID_STR){
			save(config_.path);
		}
	}


	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	EpisodeInfo
	QLearningSolver<EnvTp, PolicyType>::on_training_episode(env_type& env, uint_t episode_idx){

		auto start = std::chrono::steady_clock::now();
		EpisodeInfo info;

		// total score for the episode
		auto episode_score = 0.0;
		auto state = env.reset().observation();

		uint_t itr=0;
		for(;  itr < config_.max_num_iterations_per_episode; ++itr){


			// select an action
			auto action = policy_(q_table_, state);

			// Take a on_episode
			auto step_type_result = env.step(action);

			auto next_state = step_type_result.observation();
			auto reward = step_type_result.reward();
			auto done = step_type_result.done();

			// accumulate score
			episode_score += reward;

			if(!done){
				auto next_action = policy_(q_table_, state);
				update_q_table_(action, state, next_state, next_action, reward);
				state = next_state;
				action = next_action;
			}
			else{

				update_q_table_(action, state,
				                bitsim::consts::INVALID_ID,
				                bitsim::consts::INVALID_ID,
				                reward);


				break;
			}
		}

		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<real_t> elapsed_seconds = end-start;

		info.episode_index = episode_idx;
		info.episode_reward = config_.average_episode_reward ? episode_score / static_cast<real_t>(itr) : episode_score;
		info.episode_iterations = itr;
		info.total_time = elapsed_seconds;
		return info;
	}

	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	void
	QLearningSolver<EnvTp, PolicyType>::actions_after_episode_ends(env_type&, uint_t episode_idx,
	                                                               const EpisodeInfo& /*einfo*/){
		policy_.on_episode(episode_idx);
	}

	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	void
	QLearningSolver<EnvTp, PolicyType>::save(const std::string& filename)const{

		bitsim::utils::io::CSVWriter file_writer(filename, ',');
		file_writer.open();

		std::vector<std::string> col_names(1 + q_table_.cols());
		col_names[0] = "state_index";

		for(uint_t i = 0; i< static_cast<uint_t>(q_table_.cols()); ++i){
			col_names[i + 1] = "action_" + std::to_string(i);
		}

		file_writer.write_column_names(col_names);

		for(uint_t s=0; s < static_cast<uint_t>(q_table_.rows()); ++s){
			auto actions = maths::get_row(q_table_, s);
			auto row = std::make_tuple(s, actions);
			file_writer.write_row(row);
		}
	}


	template<envs::discrete_world_concept EnvTp, typename PolicyType>
	bitrl::rl::policies::MaxTabularPolicy
	QLearningSolver<EnvTp, PolicyType>::build_policy()const{

		bitrl::rl::policies::MaxTabularPolicy policy;
		bitrl::rl::policies::MaxTabularPolicyBuilder builder;
		builder.build_from_state_action_function(q_table_,policy);
		return policy;

	}

	template <envs::discrete_world_concept EnvTp, typename PolicyType>
	void
	QLearningSolver<EnvTp, PolicyType>::update_q_table_(const action_type& action, const state_type& cstate,
	                                                    const state_type& next_state,
	                                                    const  action_type& /*next_action*/, real_t reward){

		auto q_current = q_table_(cstate, action);
		auto q_next = next_state != bitsim::consts::INVALID_ID ? bitrl::maths::get_row_max(q_table_, next_state) : 0.0;


		auto td_target = reward + config_.gamma * q_next;
		q_table_(cstate, action) = q_current + (config_.eta * (td_target - q_current));

	}



}
}

#endif // Q_LEARNING_H
