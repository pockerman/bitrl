#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"

#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include "bitrl/rl/algorithms/pg/ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/maths/optimization/optimizer_type.h"
#include "bitrl/maths/statistics/distributions/torch_normal.h"
#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"
#include "bitrl/utils/torch_adaptor.h"

#include "bitsim/utils/io/csv_file_writer.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/envs/gymnasium/box2d/lunar_lander_env.h"

#include <boost/log/trivial.hpp>
#include <torch/torch.h>

#include <vector>
#include <iostream>
#include <unordered_map>
#include <filesystem>

namespace rl_example_11{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string EXPERIMENT_ID = "1";
const std::string POLICY = "policy.csv";

namespace F = torch::nn::functional;

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::torch_tensor_t;
using bitrl::DeviceType;
using bitrl::maths::stats::TorchNormalDist;
using bitrl::rl::algos::pg::PPOConfig;
using bitrl::rl::algos::pg::PPOSolver;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::utils::pytorch::TorchAdaptor;
	using bitrl::network::RESTRLEnvClient;

typedef  bitrl::envs::gymnasium::LunarLanderContinuousEnv env_type;

/// Layer sizes for Actor
const uint_t L1 = 8;
const uint_t L2 = 128;
const uint_t L3 = 64;
constexpr real_t LEARNING_RATE = 0.001;
const auto N_EPISODES = 2000;


// create the Action network
class ActorNetImpl: public torch::nn::Module
{
public:

    
    ActorNetImpl();
	 std::pair<torch::Tensor, torch::Tensor> forward(torch_tensor_t state);

	template<typename StateTp>
	std::tuple<std::vector<real_t>,torch_tensor_t> act(const StateTp& state);

    /// Evaluate the actor
    /// @param states //
    /// @param actions
    /// @return
    std::tuple<torch_tensor_t, torch_tensor_t, torch_tensor_t>
	evaluate(torch_tensor_t states, torch_tensor_t actions);
			   
private:

    torch::nn::Linear fc1_;
	torch::nn::Dropout dp_;
    torch::nn::Linear  fc2_;
	torch::nn::Linear  mean_layer_;
	torch::nn::Linear log_std_;

};

ActorNetImpl::ActorNetImpl()
:
torch::nn::Module(),
fc1_(torch::nn::Linear(L1, L2)),
dp_(torch::nn::Dropout(0.6)),
fc2_(torch::nn::Linear(L2, L3)),
mean_layer_(torch::nn::Linear(64, 2)),
log_std_(torch::nn::Linear(64, 2))
{
   register_module("fc1_", fc1_);
   register_module("dp_",  dp_);
   register_module("fc2_", fc2_);
   register_module("mean_layer_", mean_layer_);
   register_module("log_std_", log_std_);
}

template<typename StateTp>
std::tuple<std::vector<real_t>, torch_tensor_t>
ActorNetImpl::act(const StateTp& state){

	auto torch_state = torch::tensor(state, torch::dtype(torch::kFloat32));
	auto [mean, log_std] = this->forward(torch_state);
	auto std = torch::exp(log_std);

	auto dist = TorchNormalDist(mean, std);

	// sample an action
	auto raw_action = dist.sample();
	auto action = torch::tanh(raw_action);

	auto log_prob = -0.5 * (((raw_action - mean) / (std + 1e-8)).pow(2)
							+ 2 * log_std
							+ std::log(2 * M_PI));
	log_prob = log_prob.sum(-1); // sum over action dims

	auto log_det_jacobian = torch::log(1 - action.pow(2) + 1e-6).sum(-1);
	auto corrected_log_prob = log_prob - log_det_jacobian;

	auto clamped_action = TorchAdaptor::to_vector<real_t>(action.to(torch::kDouble));
	return {clamped_action, corrected_log_prob};
}

std::pair<torch_tensor_t, torch_tensor_t>
ActorNetImpl::forward(torch_tensor_t state){

	state = torch::relu(fc1_ -> forward(state));
	state = torch::relu(fc2_ -> forward(state));
	auto mean = mean_layer_-> forward(state);
	auto log_std = log_std_-> forward(state);
	//auto std = torch::exp(log_std);

	// Clamp for numerical stability
	log_std = torch::clamp(log_std, -20.0f, 2.0f);

	return {mean, log_std};
}

std::tuple<torch_tensor_t, torch_tensor_t, torch_tensor_t>
ActorNetImpl::evaluate(torch_tensor_t states, torch_tensor_t actions) {
	auto [mean, std] = this->forward(states);
	auto dist = TorchNormalDist(mean, std);
	auto log_probs = dist.log_prob(actions).sum(1);
	auto entropy = dist.entropy().sum(1).mean();
	return {log_probs, entropy, mean};
}


/// The Critic network 
class CriticNetImpl: public torch::nn::Module
{
public:
    CriticNetImpl();
    torch_tensor_t forward(torch_tensor_t state);
	
	template<typename StateTp>
	torch_tensor_t evaluate(const StateTp& state);
private:
    torch::nn::Linear fc1_;
    torch::nn::Linear fc2_;
    torch::nn::Linear fc3_;
};


CriticNetImpl::CriticNetImpl()
:
torch::nn::Module(),
fc1_(torch::nn::Linear(L1, L2)),
fc2_(torch::nn::Linear(L2, 256)),
fc3_(torch::nn::Linear(256, 1))
{
   register_module("fc1_", fc1_);
   register_module("fc2_", fc2_);
   register_module("fc3_", fc3_);
}

torch_tensor_t
CriticNetImpl::forward(torch_tensor_t state){

    auto output = F::relu(fc1_ -> forward(state));
    output = F::relu(fc2_ -> forward(output));
    output = fc3_ -> forward(output);
    return output;
}

template<typename StateTp>
torch_tensor_t 
CriticNetImpl::evaluate(const StateTp& state){
	
	auto torch_state = torch::tensor(state);
	return forward(torch_state);
}

TORCH_MODULE(ActorNet);
TORCH_MODULE(CriticNet);

}


int main(){

	BOOST_LOG_TRIVIAL(info)<<"Starting agent training";
    using namespace rl_example_11;

    try{
		
		// let's create a directory where we want to
        //store all the results from running a simualtion
        std::filesystem::create_directories("experiments/" + EXPERIMENT_ID);

		torch::manual_seed(42);
		RESTRLEnvClient server(SERVER_URL, true);
		
        // create the environment
        env_type env(server);

		BOOST_LOG_TRIVIAL(info)<<"Creating environment...";
        std::unordered_map<std::string, std::any> options;
		std::unordered_map<std::string, std::any> reset_options;
        env.make("v3", options, reset_options);
        env.reset();
		
        BOOST_LOG_TRIVIAL(info)<<"Done...";
		BOOST_LOG_TRIVIAL(info)<<"Number of actions="<<env.n_actions();
        
        PPOConfig ppo_config;
		ppo_config.max_itrs_per_episode = 500; 
		ppo_config.n_episodes = N_EPISODES;
		ppo_config.device_type = DeviceType::CPU;
        
		ActorNet policy;
        CriticNet critic;

        std::map<std::string, std::any> opt_options;
        opt_options.insert(std::make_pair("lr", LEARNING_RATE));

		using namespace bitrl::maths::optim::pytorch;
		using namespace bitrl::maths::optim;

        auto pytorch_ops = build_pytorch_optimizer_options(OptimizerType::ADAM,
														  opt_options);

        auto policy_optimizer = build_pytorch_optimizer(OptimizerType::ADAM,
                                                        *policy, pytorch_ops);

        auto critic_optimizer = build_pytorch_optimizer(OptimizerType::ADAM,
                                                        *critic, pytorch_ops);

        typedef PPOSolver<env_type, ActorNet, CriticNet> solver_type;

        solver_type solver(ppo_config, policy, critic,
                           policy_optimizer, critic_optimizer);

        RLSerialTrainerConfig config;
		config.n_episodes = N_EPISODES;
		config.output_msg_frequency = 20;
        
		RLSerialAgentTrainer<env_type, solver_type> trainer(config, solver);
        
		auto info = trainer.train(env);
		
		BOOST_LOG_TRIVIAL(info)<<"Training info...";
		BOOST_LOG_TRIVIAL(info)<<info;
		
		auto experiment_path = std::string("experiments/") + EXPERIMENT_ID;
		
		// write the loss values
		auto& policy_loss_vals= solver.get_monitor().policy_loss_values;
		
		bitsim::utils::io::CSVWriter loss_csv_writer(experiment_path + "/" + "policy_loss.csv",
														  bitsim::utils::io::CSVWriter::default_delimiter());
		loss_csv_writer.open();
		
		auto episode_counter = 0;
		for(uint_t i=0; i<policy_loss_vals.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, policy_loss_vals[i]};
			loss_csv_writer.write_row(row);
		}
		
		loss_csv_writer.close();
		
		// write the loss values
		auto& critic_loss_vals  = solver.get_monitor().critic_loss_values;
		
		bitsim::utils::io::CSVWriter critic_csv_writer(experiment_path + "/" + "critic_loss.csv",
														  bitsim::utils::io::CSVWriter::default_delimiter());
		critic_csv_writer.open();
		
		episode_counter = 0;
		for(uint_t i=0; i<policy_loss_vals.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, critic_loss_vals[i]};
			critic_csv_writer.write_row(row);
		}
		
		critic_csv_writer.close();

		auto& rewards  = solver.get_monitor().rewards;
		bitsim::utils::io::CSVWriter rewards_csv_writer(experiment_path + "/" + "rewards.csv",
														  bitsim::utils::io::CSVWriter::default_delimiter());
		rewards_csv_writer.open();
		
		episode_counter = 0;
		for(uint_t i=0; i<rewards.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, rewards[i]};
			rewards_csv_writer.write_row(row);
		}
		
		rewards_csv_writer.close();
		
		
		auto& episode_duration  = solver.get_monitor().episode_duration;
		bitsim::utils::io::CSVWriter episode_duration_csv_writer(experiment_path + "/" + "episode_duration.csv",
														            bitsim::utils::io::CSVWriter::default_delimiter());
		episode_duration_csv_writer.open();
		
		episode_counter = 0;
		for(uint_t i=0; i<episode_duration.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, episode_duration[i]};
			episode_duration_csv_writer.write_row(row);
		}

		episode_duration_csv_writer.close();
		BOOST_LOG_TRIVIAL(info)<<"Finished agent training";
    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){

        std::cout<<"Unknown exception occured"<<std::endl;
    }
    return 0;
}
#else
#include <iostream>

int main(){

    std::cout<<"This example requires PyTorch. Reconfigure bitrl with USE_PYTORCH fag turned ON."<<std::endl;
    return 1;
}
#endif


