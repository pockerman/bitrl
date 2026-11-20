#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/algorithms/utils.h"



#include <iostream>
#include <unordered_map>
#include <vector>

namespace example_1
{

	using namespace bitrl;
using bitrl::uint_t;
using bitrl::real_t;

}

int main() {

    using namespace example_1;
	using namespace bitrl::rl::algos;
    try{
		
		{
			std::vector<real_t> rewards(10, 1);
			auto discounted_reward = calculate_discounted_return(rewards, 1.0);
																					
			auto true_reward = bitrl::maths::sum(rewards);
																					
			std::cout<<"Discounted reward: "
					 <<discounted_reward
					 <<" expected: "
					 <<true_reward<<std::endl;
		}
				 
		{		 
			// frequently we want to calculated the discounted
			// returns for a series of steps we have taken over
			// an episode. see for example the REINFORCE algorithm
			std::vector<real_t> rewards = {0.0, 1.0, 2.0, 3.0, 4.0};
		
			auto step_discounted_reward = calculate_step_discounted_return(rewards, 0.99);
		
			const real_t expected_step_discounted_reward[] = {9.70348104, 9.801496, 
		                                                     8.8904, 6.96, 4.0};
															 
			std::cout<<"step_discounted_reward size :"<<step_discounted_reward.size()<<std::endl; 
		
			for(uint_t i=0; i<step_discounted_reward.size(); ++i){
				std::cout<<"Reward: "
						 <<step_discounted_reward[i]
						 <<" expected: "
						 <<expected_step_discounted_reward[i]<<std::endl;
			
			}
		}
		
		
    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){

        std::cout<<"Unknown exception occured"<<std::endl;
    }

   return 0;
}
