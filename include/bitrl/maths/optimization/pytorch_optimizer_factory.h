#ifndef PYTORCH_OPTIMIZER_FACTORY_H
#define PYTORCH_OPTIMIZER_FACTORY_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH


#include "bitrl/maths/optimization/optimizer_type.h"
#include <torch/torch.h>
#include <memory>
#include <map>
#include <any>
#include <string>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl {
namespace maths{
namespace optim {
namespace pytorch {


///
///
///
std::unique_ptr<torch::optim::OptimizerOptions>
build_pytorch_optimizer_options(OptimizerType type, const std::map<std::string, std::any>& options);

///
/// \brief build_pytorch_optimizer
/// \param type
/// \param model
/// \return
std::unique_ptr<torch::optim::Optimizer>
build_pytorch_optimizer(OptimizerType type, torch::nn::Module& model, const torch::optim::OptimizerOptions& options);

///
/// \brief build_pytorch_optimizer
/// \param type
/// \param model
/// \return
std::unique_ptr<torch::optim::Optimizer>
build_pytorch_optimizer(OptimizerType type, torch::nn::Module& model, std::unique_ptr<torch::optim::OptimizerOptions>& options){
    return build_pytorch_optimizer(type, model, *options.get());
}

}
}
}
}
#endif
#endif // PYTORCH_OPTIMIZER_FACTORY_H
