#ifndef DUMMY_AGENT_H
#define DUMMY_AGENT_H

#include <boost/noncopyable.hpp>

namespace bitrl
{
namespace rl
{
namespace agents
{

///
///
///
template <typename EnvType, typename PolicyType> class DummyAgent final : private boost::noncopyable
{

  public:
    typedef EnvType env_type;
    typedef typename env_type::state_type state_type;
    typedef typename env_type::action_type action_type;
    typedef PolicyType policy_type;

    ///
    /// \brief DummyAgent
    /// \param policy
    ///
    explicit DummyAgent(const policy_type &policy);

    ///
    ///
    ///
    template <typename Criteria> void play(EnvType &env, Criteria &criteria);

    ///
    /// \brief on_state
    /// \param state
    /// \return
    ///
    action_type on_state(const state_type &state);

  protected:
    ///
    /// \brief policy_
    ///
    const policy_type &policy_;
};

template <typename EnvType, typename PolicyType>
DummyAgent<EnvType, PolicyType>::DummyAgent(const policy_type &policy) : policy_(policy)
{
}

template <typename EnvType, typename PolicyType>
template <typename Criteria>
void DummyAgent<EnvType, PolicyType>::play(EnvType &env, Criteria &criteria)
{

    auto time_step = env.reset();

    while (criteria.continue_iterations())
    {

        auto action = on_state(time_step.observation());
        time_step = env.step(action);

        if (time_step.done())
        {
            time_step = env.reset();
        }
    }
}

template <typename EnvType, typename PolicyType>
typename DummyAgent<EnvType, PolicyType>::action_type
DummyAgent<EnvType, PolicyType>::on_state(const state_type &state)
{

    return policy_.on_state(state);
}

} // namespace agents

} // namespace rl

} // namespace bitrl

#endif // DUMMY_AGENT_H
