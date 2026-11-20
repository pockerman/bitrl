#include "bitrl/maths/statistics/distributions/torch_categorical.h"

#ifdef USE_PYTORCH

#include <stdexcept>
#include <cmath>

namespace bitrl {
namespace maths {
namespace stats {



TorchCategorical::TorchCategorical(torch_tensor_t probs, bool do_build_from_logits)
:
TorchDistributionBase(),
probs_(),
logits_(),
param_(),
num_events_()
{
    if(do_build_from_logits){
      build_from_logits(probs);
    }
    else{
      build_from_probabilities(probs);
    }
}


torch_tensor_t
TorchCategorical::entropy()const{
    auto p_log_p = logits_ * probs_;
    return -p_log_p.sum(-1);
}

torch_tensor_t
TorchCategorical::log_prob(torch_tensor_t value){
    value = value.to(torch::kLong).unsqueeze(-1);
    auto broadcasted_tensors = torch::broadcast_tensors({value, logits_});
    value = broadcasted_tensors[0];
    value = value.narrow(-1, 0, 1);
    return broadcasted_tensors[1].gather(-1, value).squeeze(-1);
}

torch_tensor_t
TorchCategorical::sample(c10::ArrayRef<int64_t> sample_shape){
    auto ext_sample_shape = extended_shape(sample_shape);
    auto param_shape = ext_sample_shape;
    param_shape.insert(param_shape.end(), {num_events_});
    auto exp_probs = probs_.expand(param_shape);
    torch_tensor_t probs_2d;

    if (probs_.dim() == 1 || probs_.size(0) == 1)
    {
        probs_2d = exp_probs.view({-1, num_events_});
    }
    else
    {
        probs_2d = exp_probs.contiguous().view({-1, num_events_});
    }
    auto sample_2d = torch::multinomial(probs_2d, 1, true);
    return sample_2d.contiguous().view(ext_sample_shape);
}


void
TorchCategorical::build_from_logits(torch_tensor_t logits){

    if (logits.dim() < 1){
        throw std::runtime_error("Logits tensor must have at least one dimension");
    }

    logits_ = logits - logits.logsumexp(-1, true);
    probs_ = torch::softmax(logits_, -1);

    param_ = logits;
    num_events_ = param_.size(-1);

    if (param_.dim() > 1){
        batch_shape_ = param_.sizes().vec();
        batch_shape_.resize(batch_shape_.size() - 1);
    }
}

void
TorchCategorical::build_from_probabilities(torch_tensor_t probs){

    if (probs.dim() < 1){
        throw std::runtime_error("Probabilities tensor must have at least one dimension");
    }

    probs_ = probs / probs.sum(-1, true);

    // 1.21e-7 is used as the epsilon to
    // match PyTorch's Python results as close as possible
    probs_ = probs_.clamp(1.21e-7, 1. - 1.21e-7);
    logits_ = torch::log(probs_);

    param_ = probs_;
    num_events_ = param_.size(-1);

    if (param_.dim() > 1){
        batch_shape_ = param_.sizes().vec();
        batch_shape_.resize(batch_shape_.size() - 1);
    }
}

}
}
}
#endif
