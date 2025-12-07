#include "../../src/bitrl/network/rest_rl_env_client.h"
#include <iostream>
#include <string>

int main(){

    const std::string SERVER_URL = "http://0.0.0.0:8001/api";

    bitrl::envs::RESTRLEnvClient server_wrapper(SERVER_URL);

    auto has_gym = server_wrapper.has_gymnasium();
    std::cout<<"Has environment server Gymnasium? "<<has_gym<<std::endl;

    if(has_gym){
     auto envs = server_wrapper.gymnasium_envs();

     for(auto env:envs){
         std::cout<<"Environment: "<<env<<std::endl;
     }
    }

    return 0;
}
