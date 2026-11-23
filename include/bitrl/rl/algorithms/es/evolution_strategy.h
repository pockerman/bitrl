#ifndef BITRL_RL_ALGORITHMS_ES_EVOLUTION_STRATEGY_H
#define BITRL_RL_ALGORITHMS_ES_EVOLUTION_STRATEGY_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/es/es_config.h"
#include "bitrl/rl/distributed/communicator.h"
#include "bitsim/utils/concurrency/parallel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace bitrl
{
namespace rl::algos::es
{

///
/// \brief Normal random numbers that are the same on every machine.
///
/// std::normal_distribution is left to the standard library, so two machines can draw other
/// numbers from the same seed. This draws them from std::mt19937_64, whose numbers the standard
/// fixes, with the Box-Muller transform, so that ranks on any machines make the same noise.
///
class PortableNormal
{
  public:
    explicit PortableNormal(std::uint64_t seed) : rng_(seed) {}

    real_t operator()()
    {
        if (has_spare_)
        {
            has_spare_ = false;
            return spare_;
        }
        // u1 in (0, 1], u2 in [0, 1), from the top 53 bits
        const real_t u1 = (static_cast<real_t>(rng_() >> 11) + 1.0) * 0x1.0p-53;
        const real_t u2 = static_cast<real_t>(rng_() >> 11) * 0x1.0p-53;
        const real_t r = std::sqrt(-2.0 * std::log(u1));
        const real_t a = 2.0 * M_PI * u2;
        spare_ = r * std::sin(a);
        has_spare_ = true;
        return r * std::cos(a);
    }

  private:
    std::mt19937_64 rng_;
    real_t spare_{0.0};
    bool has_spare_{false};
};

///
/// \brief What one generation of EvolutionStrategy gave
///
struct GenerationResult
{
    uint_t generation{0};
    real_t mean_fitness{0.0};    ///< over the 2 * n_pairs perturbed parameters
    real_t max_fitness{0.0};
    real_t min_fitness{0.0};
    real_t gradient_norm{0.0};   ///< of the estimated gradient, before Adam
};

///
/// \brief The evolution strategy of Salimans et al. (2017): a search for the parameters that
/// maximise a fitness, which needs only the fitness, not its gradient.
///
/// Every generation draws n_pairs directions eps_k of standard normal noise, and evaluates the
/// fitness at theta + sigma eps_k and at theta - sigma eps_k: an antithetic pair. The fitnesses
/// are replaced by their centred ranks, in [-0.5, 0.5], which ignores how far apart they are and
/// so copes with outliers; then
///
///   g = sum_k (F(theta + sigma eps_k) - F(theta - sigma eps_k)) eps_k / (2 n_pairs sigma)
///
/// estimates the gradient of the fitness smoothed by the noise, and Adam takes a step up it,
/// with weight decay.
///
/// The fitness can be anything: the return of an episode, a cost a controller should keep low
/// negated, something that is not differentiable. The evaluations of a generation do not depend
/// on each other, so step() runs them on the threads of a pool, or shares them out among the
/// ranks of a distributed run. The noise of pair k in generation g is drawn with the seed
/// task_seed(task_seed(seed, g), k), so every rank can make every direction itself, and only the
/// fitnesses travel: 2 * n_pairs numbers per generation, however many parameters there are.
/// The result does not depend on the number of threads or ranks, to the last bit.
///
/// A fitness is called as fitness(parameters, generation, pair), and must be safe to call
/// from several threads at once. Both members of a pair get the same pair index, so that they
/// can play the same episode, with the same seed: that cancels the luck of the episode out of
/// their difference.
///
class EvolutionStrategy
{
  public:
    typedef std::function<real_t(const std::vector<real_t> &, uint_t, uint_t)> fitness_type;

    EvolutionStrategy(const ESConfig &config, std::vector<real_t> initial_parameters);

    ///
    /// \brief One generation, evaluated on the threads of pool
    ///
    GenerationResult step(const fitness_type &fitness,
                          bitsim::utils::concurrency::ThreadPool &pool =
                              bitsim::utils::concurrency::default_thread_pool());

    ///
    /// \brief One generation, the pairs shared out among the ranks of communicator: rank r
    /// evaluates the pairs k with k % world_size == r, on the threads of pool, and sends their
    /// fitnesses to the others. Every rank must call it, with an EvolutionStrategy made alike;
    /// afterwards every rank has the same parameters as a run on one rank.
    ///
    GenerationResult step(const fitness_type &fitness, rl::distributed::Communicator &communicator,
                          bitsim::utils::concurrency::ThreadPool &pool =
                              bitsim::utils::concurrency::default_thread_pool());

    ///
    /// \brief The parameters: the mean of the search distribution
    ///
    const std::vector<real_t> &parameters() const noexcept { return theta_; }

    ///
    /// \brief The noise of pair k in generation generation
    ///
    std::vector<real_t> noise(uint_t generation, uint_t pair) const;

    ///
    /// \brief The generations taken so far
    ///
    uint_t generation() const noexcept { return generation_; }

    ///
    /// \brief The fitnesses of the last generation: entry 2k is pair k plus, 2k + 1 pair k minus
    ///
    const std::vector<real_t> &last_fitnesses() const noexcept { return fitnesses_; }

    const ESConfig &config() const noexcept { return config_; }

    ///
    /// \brief The centred ranks of values, in [-0.5, 0.5]; equal values share their mean rank
    ///
    static std::vector<real_t> centred_ranks(const std::vector<real_t> &values);

    ///
    /// \brief A checksum of the parameters, to tell that two runs, or two ranks, agree
    ///
    std::uint64_t checksum() const noexcept;

  private:
    // evaluate the pairs for which mine(k) is true; the others stay 0
    template <typename Mine>
    void evaluate_(const fitness_type &fitness, Mine mine,
                   bitsim::utils::concurrency::ThreadPool &pool);
    GenerationResult update_();

    ESConfig config_;
    std::vector<real_t> theta_;
    std::vector<real_t> m_;
    std::vector<real_t> v_;
    std::vector<real_t> fitnesses_;
    uint_t generation_{0};
};

inline EvolutionStrategy::EvolutionStrategy(const ESConfig &config,
                                            std::vector<real_t> initial_parameters)
    : config_(config), theta_(std::move(initial_parameters)), m_(theta_.size(), 0.0),
      v_(theta_.size(), 0.0)
{
    if (theta_.empty())
    {
        throw std::invalid_argument("EvolutionStrategy: no parameters");
    }
    if (config_.n_pairs == 0 || !(config_.sigma > 0.0))
    {
        throw std::invalid_argument("EvolutionStrategy: needs n_pairs > 0 and sigma > 0");
    }
}

inline std::vector<real_t> EvolutionStrategy::noise(uint_t generation, uint_t pair) const
{
    using bitsim::utils::concurrency::task_seed;
    PortableNormal normal(task_seed(task_seed(config_.seed, generation), pair));
    std::vector<real_t> eps(theta_.size());
    for (auto &e : eps)
    {
        e = normal();
    }
    return eps;
}

template <typename Mine>
void EvolutionStrategy::evaluate_(const fitness_type &fitness, Mine mine,
                                  bitsim::utils::concurrency::ThreadPool &pool)
{
    const uint_t n = config_.n_pairs;
    fitnesses_.assign(2 * n, 0.0);
    // one task per evaluation, so that the two of a pair can run at the same time
    bitsim::utils::concurrency::parallel_for<uint_t>(
        0, 2 * n,
        [&](uint_t i)
        {
            const uint_t k = i / 2;
            if (!mine(k))
            {
                return;
            }
            const real_t sign = (i % 2 == 0) ? 1.0 : -1.0;
            auto candidate = noise(generation_, k);
            for (std::size_t j = 0; j < candidate.size(); ++j)
            {
                candidate[j] = theta_[j] + sign * config_.sigma * candidate[j];
            }
            fitnesses_[i] = fitness(candidate, generation_, k);
        },
        pool);
}

inline GenerationResult EvolutionStrategy::step(const fitness_type &fitness,
                                                bitsim::utils::concurrency::ThreadPool &pool)
{
    evaluate_(fitness, [](uint_t) { return true; }, pool);
    return update_();
}

inline GenerationResult EvolutionStrategy::step(const fitness_type &fitness,
                                                rl::distributed::Communicator &communicator,
                                                bitsim::utils::concurrency::ThreadPool &pool)
{
    const uint_t rank = communicator.rank();
    const uint_t world = communicator.world_size();
    auto mine = [world](uint_t k, uint_t r) { return k % world == r; };
    evaluate_(fitness, [&](uint_t k) { return mine(k, rank); }, pool);

    // send only this rank's fitnesses, in the order of the pairs, and put every rank's back
    std::vector<real_t> own;
    for (uint_t k = rank; k < config_.n_pairs; k += world)
    {
        own.push_back(fitnesses_[2 * k]);
        own.push_back(fitnesses_[2 * k + 1]);
    }
    const auto all = communicator.all_gather(
        std::string(reinterpret_cast<const char *>(own.data()), own.size() * sizeof(real_t)));
    for (uint_t r = 0; r < world; ++r)
    {
        std::vector<real_t> theirs(all[r].size() / sizeof(real_t));
        std::memcpy(theirs.data(), all[r].data(), theirs.size() * sizeof(real_t));
        std::size_t i = 0;
        for (uint_t k = r; k < config_.n_pairs; k += world)
        {
            if (i + 2 > theirs.size())
            {
                throw std::runtime_error("EvolutionStrategy: a rank sent too few fitnesses");
            }
            fitnesses_[2 * k] = theirs[i++];
            fitnesses_[2 * k + 1] = theirs[i++];
        }
    }
    return update_();
}

inline std::vector<real_t> EvolutionStrategy::centred_ranks(const std::vector<real_t> &values)
{
    const std::size_t n = values.size();
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t(0));
    std::stable_sort(order.begin(), order.end(),
                     [&values](std::size_t a, std::size_t b) { return values[a] < values[b]; });
    std::vector<real_t> ranks(n, 0.0);
    if (n < 2)
    {
        return ranks;
    }
    // equal values share the mean of their ranks, so that ties make no gradient
    for (std::size_t first = 0; first < n;)
    {
        std::size_t last = first + 1;
        while (last < n && values[order[last]] == values[order[first]])
        {
            ++last;
        }
        const real_t mean_rank = 0.5 * static_cast<real_t>(first + last - 1);
        for (std::size_t r = first; r < last; ++r)
        {
            ranks[order[r]] = mean_rank / static_cast<real_t>(n - 1) - 0.5;
        }
        first = last;
    }
    return ranks;
}

inline GenerationResult EvolutionStrategy::update_()
{
    const uint_t n = config_.n_pairs;
    const std::size_t d = theta_.size();

    GenerationResult result;
    result.generation = generation_;
    result.max_fitness = *std::max_element(fitnesses_.begin(), fitnesses_.end());
    result.min_fitness = *std::min_element(fitnesses_.begin(), fitnesses_.end());
    result.mean_fitness =
        std::accumulate(fitnesses_.begin(), fitnesses_.end(), 0.0) / static_cast<real_t>(2 * n);

    const auto shaped = config_.rank_shaping ? centred_ranks(fitnesses_) : fitnesses_;

    // the gradient, the pairs added in the order of k
    std::vector<real_t> g(d, 0.0);
    for (uint_t k = 0; k < n; ++k)
    {
        const real_t w = shaped[2 * k] - shaped[2 * k + 1];
        const auto eps = noise(generation_, k);
        for (std::size_t j = 0; j < d; ++j)
        {
            g[j] += w * eps[j];
        }
    }
    real_t norm2 = 0.0;
    for (std::size_t j = 0; j < d; ++j)
    {
        g[j] /= 2.0 * static_cast<real_t>(n) * config_.sigma;
        norm2 += g[j] * g[j];
        g[j] -= config_.weight_decay * theta_[j];
    }
    result.gradient_norm = std::sqrt(norm2);

    // Adam, up the gradient
    const real_t t = static_cast<real_t>(generation_ + 1);
    const real_t c1 = 1.0 - std::pow(config_.beta1, t);
    const real_t c2 = 1.0 - std::pow(config_.beta2, t);
    for (std::size_t j = 0; j < d; ++j)
    {
        m_[j] = config_.beta1 * m_[j] + (1.0 - config_.beta1) * g[j];
        v_[j] = config_.beta2 * v_[j] + (1.0 - config_.beta2) * g[j] * g[j];
        theta_[j] += config_.learning_rate * (m_[j] / c1) /
                     (std::sqrt(v_[j] / c2) + config_.adam_epsilon);
    }
    ++generation_;
    return result;
}

inline std::uint64_t EvolutionStrategy::checksum() const noexcept
{
    std::uint64_t h = 1469598103934665603ULL; // FNV-1a
    for (const real_t x : theta_)
    {
        unsigned char bytes[sizeof(real_t)];
        std::memcpy(bytes, &x, sizeof(real_t));
        for (const unsigned char b : bytes)
        {
            h ^= b;
            h *= 1099511628211ULL;
        }
    }
    return h;
}

} // namespace rl::algos::es
} // namespace bitrl

#endif // BITRL_RL_ALGORITHMS_ES_EVOLUTION_STRATEGY_H
