#include "bitrl/envs/multi_armed_bandits/multi_armed_bandits.h"
#include "bitrl/envs/time_step_type.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/utils/bitsim_utils.h"

#include <algorithm>
#include <exception>
#include <iostream>
#include <vector>

namespace bitrl
{
namespace envs::bandits
{

const std::string MultiArmedBandits::name = "MultiArmedBandits";
std::atomic<uint_t> MultiArmedBandits::n_copies_ = 0;

MultiArmedBandits::MultiArmedBandits() : EnvBase("0", "MultiArmedBandits") { ++n_copies_; }

void MultiArmedBandits::make(const std::string &version,
                             const std::unordered_map<std::string, std::any> &options,
                             const std::unordered_map<std::string, std::any> &reset_options)
{

    auto p_itr = options.find("p");
    if (p_itr == options.end())
    {
        throw std::logic_error("option p is missing");
    }

    auto p = std::any_cast<std::vector<real_t>>(p_itr->second);

    bandits_.reserve(p.size());
    for (auto p_ : p)
    {
        bandits_.push_back(bitsim::maths::stats::BernoulliDist(p_));
    }

    auto success_reward_itr = options.find("success_reward");
    if (success_reward_itr != options.end())
    {
        success_reward_ = std::any_cast<real_t>(success_reward_itr->second);
    }
    else
    {
        success_reward_ = 1.0;
    }

    auto fail_reward_itr = options.find("fail_reward");
    if (fail_reward_itr != options.end())
    {
        fail_reward_ = std::any_cast<real_t>(fail_reward_itr->second);
    }
    else
    {
        fail_reward_ = 0.0;
    }

    this->set_version_(version);
    auto idx = bitsim::utils::uuid4();
    this->set_idx_(idx);
    this->base_type::make(version, options, reset_options);

    // the generator is seeded once, with the seed given to make() or the
    // default seed of the environments. Every pull is an episode of its own,
    // so reset() must not seed it again, or every pull would draw the same
    // number
    seed_ = reset_options.contains("seed") ? std::any_cast<uint_t>(reset_options.at("seed"))
                                           : DEFAULT_ENV_SEED;
    generator_.seed(seed_);
    this->make_created_();
}

MultiArmedBandits::time_step_type MultiArmedBandits::reset()
{
    static auto res = [](auto &bernoulli) { bernoulli.reset(); };

    std::for_each(bandits_.begin(), bandits_.end(), res);

    return MultiArmedBandits::time_step_type(TimeStepTp::FIRST, 0.0, false, 1.0);
}

MultiArmedBandits::time_step_type MultiArmedBandits::step(const action_type &action)
{

    if (action >= bandits_.size())
    {
        throw std::logic_error("Invalid action index");
    }

    // the arm pays with its probability of success
    const bool result = std::bernoulli_distribution(bandits_[action].mean())(generator_);

    if (result)
    {
        this->get_current_time_step_() =
            MultiArmedBandits::time_step_type(TimeStepTp::LAST, success_reward_, result, 1.0);
    }
    else
    {
        this->get_current_time_step_() =
            MultiArmedBandits::time_step_type(TimeStepTp::LAST, fail_reward_, result, 1.0);
    }

    return this->get_current_time_step_();
}

void MultiArmedBandits::close()
{
    bandits_.clear();
    this->EnvBase<TimeStep<bool>, MultiArmedBanditsSpace>::close();
}

} // namespace envs::bandits
} // namespace bitrl
