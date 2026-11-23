#ifndef BITRL_RL_ALGORITHMS_ES_MLP_POLICY_H
#define BITRL_RL_ALGORITHMS_ES_MLP_POLICY_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/es/evolution_strategy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace bitrl
{
namespace rl::algos::es
{

///
/// \brief A small neural network whose weights are a flat vector of parameters, for
/// EvolutionStrategy.
///
/// The layers have the sizes given, the first the size of the observation and the last the
/// number of outputs; the hidden layers use tanh, and the outputs are linear. The network holds
/// no weights: every call takes them, so one MLPPolicy serves every candidate of a generation,
/// on every thread at once. It needs no libtorch.
///
/// The parameters are, layer after layer, the weights row by row (an output's weights over the
/// inputs) and then the biases.
///
class MLPPolicy
{
  public:
    explicit MLPPolicy(std::vector<uint_t> layer_sizes);

    ///
    /// \brief How many parameters the network has
    ///
    uint_t n_parameters() const noexcept { return n_parameters_; }

    const std::vector<uint_t> &layer_sizes() const noexcept { return sizes_; }

    ///
    /// \brief Weights drawn from N(0, 1 / fan_in), biases 0
    ///
    std::vector<real_t> initial_parameters(uint_t seed) const;

    ///
    /// \brief The outputs of the network for observation x with parameters
    ///
    std::vector<real_t> forward(const std::vector<real_t> &parameters,
                                const std::vector<real_t> &x) const;

    ///
    /// \brief The output with the largest value: the action of a discrete action space
    ///
    uint_t act_discrete(const std::vector<real_t> &parameters, const std::vector<real_t> &x) const;

    ///
    /// \brief scale * tanh(outputs): the action of a bounded continuous action space
    ///
    std::vector<real_t> act_continuous(const std::vector<real_t> &parameters,
                                       const std::vector<real_t> &x, real_t scale = 1.0) const;

  private:
    std::vector<uint_t> sizes_;
    uint_t n_parameters_{0};
};

inline MLPPolicy::MLPPolicy(std::vector<uint_t> layer_sizes) : sizes_(std::move(layer_sizes))
{
    if (sizes_.size() < 2 ||
        std::any_of(sizes_.begin(), sizes_.end(), [](uint_t s) { return s == 0; }))
    {
        throw std::invalid_argument("MLPPolicy: needs at least two layers, none empty");
    }
    for (std::size_t l = 0; l + 1 < sizes_.size(); ++l)
    {
        n_parameters_ += sizes_[l + 1] * (sizes_[l] + 1);
    }
}

inline std::vector<real_t> MLPPolicy::initial_parameters(uint_t seed) const
{
    PortableNormal normal(seed);
    std::vector<real_t> p;
    p.reserve(n_parameters_);
    for (std::size_t l = 0; l + 1 < sizes_.size(); ++l)
    {
        const real_t scale = 1.0 / std::sqrt(static_cast<real_t>(sizes_[l]));
        for (uint_t i = 0; i < sizes_[l + 1] * sizes_[l]; ++i)
        {
            p.push_back(scale * normal());
        }
        p.insert(p.end(), sizes_[l + 1], 0.0);
    }
    return p;
}

inline std::vector<real_t> MLPPolicy::forward(const std::vector<real_t> &parameters,
                                              const std::vector<real_t> &x) const
{
    if (parameters.size() != n_parameters_ || x.size() != sizes_.front())
    {
        throw std::invalid_argument("MLPPolicy: wrong number of parameters or inputs");
    }
    std::vector<real_t> in = x;
    std::size_t at = 0;
    for (std::size_t l = 0; l + 1 < sizes_.size(); ++l)
    {
        const uint_t n_in = sizes_[l];
        const uint_t n_out = sizes_[l + 1];
        const real_t *w = parameters.data() + at;
        const real_t *b = w + n_out * n_in;
        std::vector<real_t> out(n_out);
        for (uint_t o = 0; o < n_out; ++o)
        {
            real_t s = b[o];
            for (uint_t i = 0; i < n_in; ++i)
            {
                s += w[o * n_in + i] * in[i];
            }
            out[o] = (l + 2 < sizes_.size()) ? std::tanh(s) : s;
        }
        at += n_out * (n_in + 1);
        in = std::move(out);
    }
    return in;
}

inline uint_t MLPPolicy::act_discrete(const std::vector<real_t> &parameters,
                                      const std::vector<real_t> &x) const
{
    const auto out = forward(parameters, x);
    return static_cast<uint_t>(std::max_element(out.begin(), out.end()) - out.begin());
}

inline std::vector<real_t> MLPPolicy::act_continuous(const std::vector<real_t> &parameters,
                                                     const std::vector<real_t> &x,
                                                     real_t scale) const
{
    auto out = forward(parameters, x);
    for (auto &o : out)
    {
        o = scale * std::tanh(o);
    }
    return out;
}

///
/// \brief The undiscounted return of one episode of env, reset with seed, acting with act,
/// at most max_steps steps
///
template <typename EnvType, typename Act>
real_t episode_return(EnvType &env, Act &&act, uint_t seed, uint_t max_steps)
{
    auto step = env.reset(seed);
    real_t total = 0.0;
    for (uint_t t = 0; t < max_steps && !step.done(); ++t)
    {
        step = env.step(act(step.observation()));
        total += step.reward();
    }
    return total;
}

} // namespace rl::algos::es
} // namespace bitrl

#endif // BITRL_RL_ALGORITHMS_ES_MLP_POLICY_H
