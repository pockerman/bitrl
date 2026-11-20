#include "cuberl/base/cuberl_config.h"

#ifdef USE_PYTORCH

#include "cuberl/base/cuberl_types.h"
#include "cuberl/maths/statistics/distributions/torch_categorical.h"
#include "cuberl/rl/trainers/rl_serial_agent_trainer.h"
#include "cuberl/rl/algorithms/pg/a2c.h"
#include "cuberl/rl/algorithms/pg/a2c_config.h"
#include "cuberl/maths/optimization/optimizer_type.h"
#include "cuberl/maths/optimization/pytorch_optimizer_factory.h"

#include "bitrl/utils/io/csv_file_writer.h"
#include "cuberl/network/rest_rl_env_client.h"
#include "cuberl/envs/gymnasium/classic_control/cart_pole_env.h"

#include <boost/log/trivial.hpp>
#include <torch/torch.h>

#include <iostream>
#include <unordered_map>
#include <filesystem>

namespace rl_example_11{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string EXPERIMENT_ID = "1";
const std::string POLICY = "policy.csv";

namespace F = torch::nn::functional;

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::torch_tensor_t;
using cuberl::DeviceType;
using cuberl::maths::stats::TorchCategorical;
using cuberl::rl::algos::pg::A2CConfig;
using cuberl::rl::algos::pg::A2CSolver;
using cuberl::rl::RLSerialAgentTrainer;
using cuberl::rl::RLSerialTrainerConfig;
	using cuberl::network::RESTRLEnvClient;

typedef  cuberl::envs::gymnasium::CartPole env_type;

/// Layer sizes for Actor
const uint_t L1 = 4;
const uint_t L2 = 128;
const uint_t L3 = 2;
const real_t LEARNING_RATE = 0.01;
const auto N_EPISODES = 2000;


// create the Action network
class ActorNetImpl: public torch::nn::Module
{
public:

    
    ActorNetImpl();
	torch_tensor_t forward(torch_tensor_t state);
	template<typename StateTp>
	std::tuple<uint_t, torch_tensor_t> act(const StateTp& state);
			   
private:

    torch::nn::Linear fc1_;
	torch::nn::Dropout dp_;
    torch::nn::Linear  fc2_;

};

ActorNetImpl::ActorNetImpl()
:
torch::nn::Module(),
fc1_(torch::nn::Linear(L1, L2)),
dp_(torch::nn::Dropout(0.6)),
fc2_(torch::nn::Linear(L2, L3))
{
   register_module("fc1_", fc1_);
   register_module("dp_",  dp_);
   register_module("fc2_", fc2_);
}

template<typename StateTp>
std::tuple<uint_t, torch_tensor_t> 
ActorNetImpl::act(const StateTp& state){
	
	auto torch_state = torch::tensor(state);
    auto probs = forward(torch_state);
	
	auto m = TorchCategorical(probs, false);
	
	auto action = m.sample();
    return std::make_tuple(action.item().toLong(), 
	                       m.log_prob(action));
}


torch_tensor_t
ActorNetImpl::forward(torch_tensor_t state){
	
	state = fc1_->forward(state);
	state = dp_ -> forward(state);
		
	state = F::relu(state);
    state = fc2_->forward(state);
    return F::softmax(state,  F::SoftmaxFuncOptions(0));
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
fc1_(torch::nn::Linear(4, 128)),
fc2_(torch::nn::Linear(128, 256)),
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
        env.make("v1", options, reset_options);
        env.reset();
		
        BOOST_LOG_TRIVIAL(info)<<"Done...";
		BOOST_LOG_TRIVIAL(info)<<"Number of actions="<<env.n_actions();
        
        A2CConfig a2c_config;
		a2c_config.max_itrs_per_episode = 500; 
		a2c_config.n_episodes = N_EPISODES;
		a2c_config.device_type = DeviceType::CPU;
        
		ActorNet policy;
        CriticNet critic;

        std::map<std::string, std::any> opt_options;
        opt_options.insert(std::make_pair("lr", LEARNING_RATE));

		using namespace cuberl::maths::optim::pytorch;
		using namespace cuberl::maths::optim;

        auto pytorch_ops = build_pytorch_optimizer_options(OptimizerType::ADAM,
														  opt_options);

        auto policy_optimizer = build_pytorch_optimizer(OptimizerType::ADAM,
                                                        *policy, pytorch_ops);

        auto critic_optimizer = build_pytorch_optimizer(OptimizerType::ADAM,
                                                        *critic, pytorch_ops);

        typedef A2CSolver<env_type, ActorNet, CriticNet> solver_type;

        solver_type solver(a2c_config, policy, critic,
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
		auto& policy_loss_vals  = solver.get_monitor().policy_loss_values;
		
		bitrl::utils::io::CSVWriter loss_csv_writer(experiment_path + "/" + "policy_loss.csv",
														  bitrl::utils::io::CSVWriter::default_delimiter());
		loss_csv_writer.open();
		
		auto episode_counter = 0;
		for(uint_t i=0; i<policy_loss_vals.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, policy_loss_vals[i]};
			loss_csv_writer.write_row(row);
		}
		
		loss_csv_writer.close();
		
		// write the loss values
		auto& critic_loss_vals  = solver.get_monitor().critic_loss_values;
		
		bitrl::utils::io::CSVWriter critic_csv_writer(experiment_path + "/" + "critic_loss.csv",
														  bitrl::utils::io::CSVWriter::default_delimiter());
		critic_csv_writer.open();
		
		episode_counter = 0;
		for(uint_t i=0; i<policy_loss_vals.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, critic_loss_vals[i]};
			critic_csv_writer.write_row(row);
		}
		
		critic_csv_writer.close();
		
		
		
		auto& rewards  = solver.get_monitor().rewards;
		bitrl::utils::io::CSVWriter rewards_csv_writer(experiment_path + "/" + "rewards.csv",
														  bitrl::utils::io::CSVWriter::default_delimiter());
		rewards_csv_writer.open();
		
		episode_counter = 0;
		for(uint_t i=0; i<rewards.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, rewards[i]};
			rewards_csv_writer.write_row(row);
		}
		
		rewards_csv_writer.close();
		
		
		auto& episode_duration  = solver.get_monitor().episode_duration;
		bitrl::utils::io::CSVWriter episode_duration_csv_writer(experiment_path + "/" + "episode_duration.csv",
														            bitrl::utils::io::CSVWriter::default_delimiter());
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

    std::cout<<"This example requires PyTorch. Reconfigure cuberl with USE_PYTORCH fag turned ON."<<std::endl;
    return 1;
}
#endif
