#ifndef TORCH_CATEGORICAL_H
#define TORCH_CATEGORICAL_H
/**
 * Major implementation is taken from  https://github.com/Omegastick/pytorch-cpp-rl/tree/master
 *
 */

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/statistics/distributions/torch_distribution.h"

#include <torch/torch.h>

namespace bitrl
{
namespace maths::stats
{

class TorchCategorical final : public TorchDistributionBase
{

  public:
    ///
    /// \brief Default constructor
    ///
    ///
    TorchCategorical() = default;

    ///
    /// \brief Build the categorical distribution from given
    ///	torch_tensor_t of probabilities. if the flag
    ///	build_from_logits, then the given tensor is assumed
    ///	to represent logits
    ///
    TorchCategorical(torch_tensor_t probs, bool do_build_from_logits = false);

    ///
    /// \brief ~TorchCategorical. Destructor
    ///
    ~TorchCategorical() override = default;

    ///
    /// \brief entropy
    /// \return
    ///
    virtual torch_tensor_t entropy() const override;

    ///
    /// \brief log_prob
    /// \param value
    /// \return
    ///
    virtual torch_tensor_t log_prob(torch_tensor_t value) override;

    ///
    ///
    ///
    virtual torch_tensor_t sample(c10::ArrayRef<int64_t> sample_shape = {}) override;

    ///
    /// \brief build the distribution form logits
    ///
    void build_from_logits(torch_tensor_t logits);

    ///
    /// \brief build the distribution from probabilities
    ///
    void build_from_probabilities(torch_tensor_t probs);

    ///
    /// \brief get_logits
    ///
    torch_tensor_t get_logits() const { return logits_; }

    ///
    /// \brief get_probs
    /// \return
    ///
    torch_tensor_t get_probs() const { return probs_; }

  private:
    torch_tensor_t probs_;
    torch_tensor_t logits_;
    torch_tensor_t param_;
    int num_events_;
};
} // namespace maths::stats
} // namespace bitrl
#endif
#endif
