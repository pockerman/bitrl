#include "bitrl/bitrl_consts.h"
#include "bitrl/envs/multi_armed_bandits/multi_armed_bandits.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/utils/bitrl_utils.h"

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

MultiArmedBandits::MultiArmedBandits()
    : EnvBase(0, "MultiArmedBandits")
{
    ++n_copies_;
}

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
        bandits_.push_back(utils::maths::stats::BernoulliDist(p_));
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
    if (success_reward_itr != options.end())
    {
        fail_reward_ = std::any_cast<real_t>(fail_reward_itr->second);
    }
    else
    {
        fail_reward_ = 0.0;
    }

    this->set_version_(version);
    auto idx = utils::uuid4();
    this->set_idx_(idx);
    this->base_type::make(version, options, reset_options);
    this->make_created_();
}

MultiArmedBandits::time_step_type MultiArmedBandits::reset()
{
    seed_ = 42;

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

    auto result = bandits_[action].sample();
    if (seed_ != consts::INVALID_ID)
    {
        result = bandits_[action].sample(seed_);
    }

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

}
} // namespace bitrl
