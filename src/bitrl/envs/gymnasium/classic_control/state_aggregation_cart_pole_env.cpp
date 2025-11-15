#include "bitrl/envs/gymnasium/classic_control/state_aggregation_cart_pole_env.h"
//#include "rlenvs/time_step_type.h"
//#include "rlenvs/time_step.h"

namespace bitrl{
namespace envs{
namespace gymnasium{

/*StateAggregationCartPole::StateAggregationCartPole(const std::string version, obj_t main_namespace, uint_t n_states,
                             uint_t state_idx, const StateAggregationCartPoleBoundaries& boundaries)
    :
      StateAggregationEnvBase<CartPole, std::tuple<uint_t, uint_t, uint_t, uint_t>>(version, main_namespace, n_states),
      state_idx_(state_idx),
      boundaries_(boundaries),
      pole_theta_space_(),
      pole_theta_velocity_space_(),
      cart_pos_space_(),
      cart_vel_space_()
{}

StateAggregationCartPole::time_step_type
StateAggregationCartPole::reset(){

    auto step = env_.reset();

    auto discrete_step = get_state(step.observation());

    current_state_ = time_step_type(TimeStepTp::FIRST, step.reward(), discrete_step, 1.0);
    return current_state_;
}

void
StateAggregationCartPole::make(){

    this->StateAggregationEnvBase<CartPole, std::tuple<uint_t, uint_t, uint_t, uint_t>>::make();
    create_states_();

}

StateAggregationCartPole::time_step_type
StateAggregationCartPole::step(const action_type& action){

    auto step = env_.step(action);

    auto discrete_step = get_state(step.observation());

    current_state_ = time_step_type(step.type(), step.reward(), discrete_step, 1.0);
    return current_state_;
}


StateAggregationCartPole::state_type
StateAggregationCartPole::get_state(const CartPole::state_type& obs)const{

    auto cart_x = obs[0];
    auto cart_x_dot = obs[1];
    auto cart_theta = obs[2];
    auto cart_theta_dot = obs[3];

    if(state_idx_ == 4){
        auto cart_x_int = digitize(cart_x, cart_pos_space_);
        auto cart_x_dot_int = digitize(cart_x_dot, cart_vel_space_);
        auto cart_theta_int = digitize(cart_theta, pole_theta_space_);
        auto cart_theta_dot_int = digitize(cart_theta_dot, pole_theta_velocity_space_);

        return std::make_tuple(cart_x_int, cart_x_dot_int, cart_theta_int, cart_theta_dot_int);
    }
    else if (state_idx_ == 0 ){
        return std::make_tuple(digitize(cart_x, cart_pos_space_), this->n_states() + 1, this->n_states() + 1, this->n_states() + 1);
    }
    else if (state_idx_ == 1){
        return std::make_tuple(this->n_states() + 1, digitize(cart_x_dot, cart_vel_space_), this->n_states() + 1, this->n_states() + 1);
    }
    else if( state_idx_ == 2 ){
        return std::make_tuple(this->n_states() + 1, this->n_states() + 1, digitize(cart_theta, pole_theta_space_), this->n_states() + 1);
    }
    else if (state_idx_ == 3){
        return std::make_tuple(this->n_states() + 1, this->n_states() + 1, this->n_states() + 1, digitize(cart_theta_dot, pole_theta_velocity_space_));
    }
}

void
StateAggregationCartPole::create_bins(){

    if(state_idx_ == 4){
         build_pole_theta_velocity_space_();
         build_pole_theta_space_();
         build_cart_position_space_();
         build_cart_velocity_space_();
    }
    else if(state_idx_ == 2 ){
        build_pole_theta_velocity_space_();
    }
    else if( state_idx_ == 0 ){
        build_cart_position_space_();
    }
    else if( state_idx_ == 1){
        build_cart_velocity_space_();
    }
    else if( state_idx_ == 3){
        build_pole_theta_velocity_space_();
    }
}

void
StateAggregationCartPole::create_states_(){

    if(state_idx_ == 4){
                for(uint_t i=0; i < cart_pos_space_.size() + 1; ++i){
                    for(uint_t j=0; j < cart_vel_space_.size() + 1; ++j){
                        for(uint_t k=0; k < pole_theta_space_.size() + 1; ++k){
                            for(uint_t l=0; l < pole_theta_velocity_space_.size() + 1; ++l){
                                discrete_observation_space_.push_back(std::make_tuple(i, j, k, l));
                            }
                        }
                    }
                }
    }
    else{
        for(uint_t l=0; l < n_states() + 1; ++l){
            discrete_observation_space_.push_back(std::make_tuple(l, 0, 0, 0));
        }
    }


}

void
StateAggregationCartPole::build_cart_position_space_(){

    auto low = boundaries_.cart_pole_pos.first;
    auto high = boundaries_.cart_pole_pos.second;
    cart_pos_space_ = linspace(low, high, this->n_states());
}

void
StateAggregationCartPole::build_cart_velocity_space_(){

    auto low = boundaries_.cart_pole_vel.first;
    auto high = boundaries_.cart_pole_vel.second;
    cart_vel_space_ = linspace(low, high, this->n_states());
}

void
StateAggregationCartPole::build_pole_theta_space_(){

    auto low = boundaries_.cart_pole_theta.first;
    auto high = boundaries_.cart_pole_theta.second;
    pole_theta_space_ = linspace(low, high, this->n_states());
}

void
StateAggregationCartPole::build_pole_theta_velocity_space_(){

    auto low = boundaries_.cart_pole_theta_vel.first;
    auto high = boundaries_.cart_pole_theta_vel.second;
    pole_theta_velocity_space_ = linspace(low, high, this->n_states());
}
*/

}
}
}
