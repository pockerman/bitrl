#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include <exception>

namespace bitrl{
namespace maths{
namespace optim {
namespace pytorch{

std::unique_ptr<torch::optim::OptimizerOptions>
build_pytorch_optimizer_options(OptimizerType type, const std::map<std::string, std::any>& options){

    std::unique_ptr<torch::optim::OptimizerOptions> ptr;

    auto lr = std::any_cast<real_t>(options.find("lr")->second);

    switch(type){

        case OptimizerType::GD:
        {
             throw std::logic_error("PyTorch does not implement GD");
        }
        case OptimizerType::SGD:
        {

            ptr = std::make_unique<torch::optim::SGDOptions>(lr);
            break;
        }
        case OptimizerType::ADAM:
        {
            ptr = std::make_unique<torch::optim::AdamOptions>(lr);
            break;
        }
        case OptimizerType::RSPROP:
        {
            ptr = std::make_unique<torch::optim::RMSpropOptions>(lr);
            break;
        }
        default:
        {
          throw std::logic_error("Invalid PyTorch optimizer type");
        }
    }
    return ptr;
}


std::unique_ptr<torch::optim::Optimizer>
build_pytorch_optimizer(OptimizerType type, torch::nn::Module& model, const torch::optim::OptimizerOptions& options){

    std::unique_ptr<torch::optim::Optimizer> ptr;

    switch(type){

        case OptimizerType::GD:
        {
             throw std::logic_error("PyTorch does not implement GD");
        }
        case OptimizerType::SGD:
        {
            ptr = std::make_unique<torch::optim::SGD>(model.parameters(),
                                                        torch::optim::SGDOptions{options.get_lr()});
            break;
        }
        case OptimizerType::ADAM:
        {
            ptr = std::make_unique<torch::optim::Adam>(model.parameters(),
                                                   torch::optim::AdamOptions{options.get_lr()});
            break;
        }
        case OptimizerType::RSPROP:
        {
            ptr = std::make_unique<torch::optim::RMSprop>(model.parameters(),
                                                      torch::optim::RMSpropOptions{options.get_lr()});
            break;
        }
        default:
        {
          throw std::logic_error("Invalid PyTorch optimizer type");
        }

    }

    return ptr;
}

}

}
}
}
#endif
