#include "bitrl/utils/io/tensor_board_server/tensorboard_server.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/extern/HTTPRequest.hpp"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <exception>

namespace bitrl{
namespace utils{
namespace io {

TensorboardServer::TensorboardServer(const std::string& api_url )
:
api_url_(api_url),
log_dir_(bitrl::consts::INVALID_STR),
api_str_("/tensorboard-api")
{}


void
TensorboardServer::init(const std::string& log_dir){

    const auto request_url = std::string(get_server_address()) + api_str_ + "/init";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;
    j["log_dir_path"] = log_dir;

    auto body = j.dump();
    const auto response = request.send("POST", body);

    if(response.status.code != 201){
        throw std::runtime_error("TensorboardServer failed to initialize");
    }

    // set the log_dir
    log_dir_ = log_dir;

}

void
TensorboardServer::close(){

    if(log_dir_ == bitrl::consts::INVALID_STR){
            return;
    }

    const auto request_url = std::string(get_server_address()) + api_str_ + + "/close";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;

    auto body = j.dump();
    const auto response = request.send("POST", body);
    log_dir_ = bitrl::consts::INVALID_STR;

}

void
TensorboardServer::add_scalar(const std::string& tag, real_t value, uint_t step_idx)const{

    if(log_dir_ == bitrl::consts::INVALID_STR){
            throw std::runtime_error("TensorboardServer is not initialized. Have you called init?");
    }

    const auto request_url = std::string(get_server_address()) + api_str_ + + "/add-scalar";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;
    j["tag"]=tag;
    j["scalar_value"] = value;

    if(step_idx != bitrl::consts::INVALID_ID){
      j["global_step"] = step_idx;
    }

    auto body = j.dump();
    const auto response = request.send("POST", body);

     if(response.status.code != 201){
        auto msg = "TensorboardServer failed to add_scalar with tag=" + tag;
        throw std::runtime_error(msg);
    }
}

void
TensorboardServer::add_scalars(const std::string& main_tag,
                     const std::unordered_map<std::string, real_t>& values,
                     uint_t step_idx)const{

    if(log_dir_ == bitrl::consts::INVALID_STR){
            throw std::runtime_error("TensorboardServer is not initialized. Have you called init?");
    }

    const auto request_url = std::string(get_server_address()) + api_str_ + + "/add-scalars";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;
    j["main_tag"]=main_tag;
    j["tag_scalar_dict"] = values;

    if(step_idx != bitrl::consts::INVALID_ID){
      j["global_step"] = step_idx;
    }

    auto body = j.dump();
    const auto response = request.send("POST", body);

     if(response.status.code != 201){
        auto msg = "TensorboardServer failed to add_scalar with main_tag=" + main_tag;
        throw std::runtime_error(msg);
    }

}

void
TensorboardServer::add_text(const std::string& tag,
                            const std::string& text,
                            uint_t step_idx)const{


    if(log_dir_ == bitrl::consts::INVALID_STR){
            throw std::runtime_error("TensorboardServer is not initialized. Have you called init?");
    }

    const auto request_url = std::string(get_server_address()) + api_str_ + + "/add-scalars";
    http::Request request{request_url};

    using json = nlohmann::json;
    json j;
    j["tag"] = tag;
    j["text_string"] = text;

    if(step_idx != bitrl::consts::INVALID_ID){
      j["global_step"] = step_idx;
    }

    auto body = j.dump();
    const auto response = request.send("POST", body);

     if(response.status.code != 201){
        auto msg = "TensorboardServer failed to add_text with tag=" + tag;
        throw std::runtime_error(msg);
    }


}

}
}
}
