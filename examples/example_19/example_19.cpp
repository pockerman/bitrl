/**
 * This example illustrates how to use the REINFORCE algorithm
 * on the CartPole environment from Gymnasium
 **/
#include "cuberl/base/cuberl_config.h"

#ifdef USE_PYTORCH

#include "cuberl/base/cuberl_types.h"
#include "cuberl/rl/algorithms/pg/reinforce.h"
#include "cuberl/rl/algorithms/pg/reinforce_config.h"
#include "cuberl/rl/trainers/rl_serial_agent_trainer.h"
#include "cuberl/maths/optimization/optimizer_type.h"
#include "cuberl/maths/optimization/pytorch_optimizer_factory.h"
#include "cuberl/maths/statistics/distributions/torch_categorical.h"

#include "bitrl/utils/io/csv_file_writer.h"
#include "cuberl/network/rest_rl_env_client.h"
#include "cuberl/envs/gymnasium/classic_control/cart_pole_env.h"

#include <boost/log/trivial.hpp>

#include <torch/torch.h>

#include <unordered_map>
#include <iostream>
#include <string>
#include <any>
#include <filesystem>
#include <map>

namespace rl_example_13{


const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string EXPERIMENT_ID = "2";
const std::string POLICY = "policy.csv";

namespace F = torch::nn::functional;

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::float_t;
using cuberl::int_t;
using cuberl::torch_tensor_t;
using cuberl::DeviceType;
using cuberl::rl::algos::pg::ReinforceSolver;
using cuberl::rl::algos::pg::ReinforceConfig;
using cuberl::rl::RLSerialAgentTrainer;
using cuberl::rl::RLSerialTrainerConfig;
using cuberl::maths::stats::TorchCategorical;
	using cuberl::network::RESTRLEnvClient;
using cuberl::envs::gymnasium::CartPole;


const uint_t L1 = 4;
const uint_t L2 = 128;
const uint_t L3 = 2;
const real_t LEARNING_RATE = 0.01;


// The class that models the Policy network to train
class PolicyNetImpl: public torch::nn::Module
{
public:

	///
	/// \brief Constructor
	///
    PolicyNetImpl();
	
	// To execute the network in C++, 
	// we simply call the forward() method
    torch_tensor_t forward(torch_tensor_t state);

	///
	/// \brief act Every policy network should expose
	/// an act function that takes a StateTp and returns
	/// an std::tuple<uint_t, torch_tensor_t>
	///
    template<typename StateTp>
    std::tuple<uint_t, torch_tensor_t> act(const StateTp& state);
	
	void make_play(){is_playing_ = true;}
	
private:

   torch::nn::Linear fc1_;
   torch::nn::Dropout dp_;
   torch::nn::Linear fc2_;
   bool is_playing_{false};
};


PolicyNetImpl::PolicyNetImpl()
    :
      fc1_(torch::nn::Linear(L1, L2)),
	  dp_(torch::nn::Dropout(0.6)),
      fc2_(torch::nn::Linear(L2, L3))
{
    register_module("fc1", fc1_);
	register_module("dp", dp_);
    register_module("fc2", fc2_);
}


torch_tensor_t
PolicyNetImpl::forward(torch_tensor_t x){

	x = fc1_->forward(x);
	
	if(!is_playing_){
		x = dp_ -> forward(x);
		
	}
    x = F::relu(x);
    x = fc2_->forward(x);
    return F::softmax(x,  F::SoftmaxFuncOptions(0));
}


template<typename StateTp>
std::tuple<uint_t, torch_tensor_t>
PolicyNetImpl::act(const StateTp& state){

    auto torch_state = torch::tensor(state);
    auto probs = forward(torch_state);
	
    auto m = TorchCategorical(probs, false);
	
    auto action = m.sample();
    return std::make_tuple(action.item().toLong(), 
	                       m.log_prob(action));

}

TORCH_MODULE(PolicyNet);

typedef CartPole env_type;
typedef PolicyNet policy_type;
typedef ReinforceSolver<env_type, PolicyNet> solver_type;
}


int main(){
	
	BOOST_LOG_TRIVIAL(info)<<"Starting agent training";
    using namespace rl_example_13;

    try{

        // let's create a directory where we want to
        //store all the results from running a simualtion
        std::filesystem::create_directories("experiments/" + EXPERIMENT_ID);
        torch::manual_seed(42);

		BOOST_LOG_TRIVIAL(info)<<"Creating environment...";
		
		RESTRLEnvClient server(SERVER_URL, true);
        auto env = CartPole(server);
	
        std::unordered_map<std::string, std::any> options;
    	std::unordered_map<std::string, std::any> reset_options;
        // with Gymnasium v0 is not working
        env.make("v1", options, reset_options);
        env.reset();

        BOOST_LOG_TRIVIAL(info)<<"Done...";
		BOOST_LOG_TRIVIAL(info)<<"Number of actions="<<env.n_actions();

        PolicyNet policy;

		// configuration for the REINFORCE solver
		ReinforceConfig opts;
		
		const auto N_EPISODES = 2000;
								
		opts.gamma = 0.999;
		opts.normalize_rewards = false;
		opts.n_episodes = N_EPISODES;
		opts.max_itrs_per_episode = 500; 
		opts.device_type = DeviceType::CPU;

        std::map<std::string, std::any> opt_options;
        opt_options.insert(std::make_pair("lr", LEARNING_RATE));

		using namespace cuberl::maths;
        auto pytorch_ops = optim::pytorch::build_pytorch_optimizer_options(optim::OptimizerType::ADAM,
																		   opt_options);

        auto policy_optimizer = optim::pytorch::build_pytorch_optimizer(optim::OptimizerType::ADAM,
																		*policy, pytorch_ops);

		
        solver_type solver(opts, policy, policy_optimizer);
		
        RLSerialTrainerConfig config;
        config.n_episodes = N_EPISODES;
        config.output_msg_frequency = 20;
        RLSerialAgentTrainer<env_type, solver_type> trainer(config, solver);
		
		
        trainer.train(env);

        auto info = trainer.train(env);
        BOOST_LOG_TRIVIAL(info)<<"Training info...";
		BOOST_LOG_TRIVIAL(info)<<info;

        // save the rewards per episode for visualization
        // purposes
		auto experiment_path = std::string("experiments/") + EXPERIMENT_ID;
        
        // save the policy also so that we can load it and check
        // use it
        auto policy_model_filename = experiment_path + std::string("/reinforce_cartpole_policy.pth");
        
		
		torch::save(policy, policy_model_filename + std::string("policy.pth"));
		
		// or we can use serialize
        torch::serialize::OutputArchive archive;
        policy->save(archive);
        archive.save_to(policy_model_filename);
		
		// write the loss values
		auto& loss_vals  = solver.get_monitor().policy_loss_values;
		
		
		BOOST_LOG_TRIVIAL(info)<<"Loss values size: "<<loss_vals.size();
		bitrl::utils::io::CSVWriter loss_csv_writer(experiment_path + "/" + "loss.csv",
														  bitrl::utils::io::CSVWriter::default_delimiter());
		loss_csv_writer.open();
		
		auto episode_counter = 0;
		for(uint_t i=0; i<loss_vals.size(); ++i){
			std::tuple<uint_t, real_t> row = {episode_counter++, loss_vals[i]};
			loss_csv_writer.write_row(row);
		}
		
		loss_csv_writer.close();
		
		
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
    return 0;
}
#endif
