// Evolution strategies swing a pole up and balance it: a policy with a continuous force,
// learned from the returns of episodes alone, with the episodes of a generation on all the
// threads.
#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/classic_control/cart_pole_swing_up.h"
#include "bitrl/rl/algorithms/es/evolution_strategy.h"
#include "bitrl/rl/algorithms/es/mlp_policy.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::classic_control::CartPoleSwingUp;
using bitsim::utils::concurrency::task_seed;
using bitsim::utils::concurrency::ThreadPool;
using namespace bitrl::rl::algos::es;

const uint_t MAX_STEPS = 500; // 10 s
const uint_t CHECK_AT = 50;   // the generations section 2 repeats on one thread

// the policy: [x, x_dot, sin, cos, theta_dot], scaled to about [-1, 1], -> 16 tanh -> force
const MLPPolicy NET({5, 16, 1});

std::vector<real_t> scaled(const std::vector<real_t> &obs)
{
    return {obs[0] / 2.4, obs[1] / 3.0, obs[2], obs[3], obs[4] / 10.0};
}

CartPoleSwingUp make_env()
{
    CartPoleSwingUp env;
    env.make("v1", {{"max_steps", MAX_STEPS}}, {});
    return env;
}

auto actor(const std::vector<real_t> &p)
{
    return [&p](const std::vector<real_t> &obs)
    { return NET.act_continuous(p, scaled(obs), 10.0); };
}

// the fitness of parameters: the return of one episode; both members of pair k of generation
// g start alike
real_t fitness(const std::vector<real_t> &p, uint_t g, uint_t k)
{
    auto env = make_env();
    return episode_return(env, actor(p), task_seed(g, k), MAX_STEPS);
}

// small output weights, so that the first forces are gentle, not all +-10 N
std::vector<real_t> initial_parameters()
{
    auto p = NET.initial_parameters(1);
    for (std::size_t j = p.size() - (16 + 1); j < p.size(); ++j)
    {
        p[j] *= 0.1;
    }
    return p;
}

ESConfig config()
{
    ESConfig c;
    c.n_pairs = 32;
    c.sigma = 0.1;
    c.learning_rate = 0.03;
    return c;
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

struct Test
{
    real_t ret{0.0};
    real_t swing_up_time{-1.0}; // the first time the pole is up; -1 if never
    real_t up_last_5s{0.0};     // the fraction of the last 5 s with the pole up
    real_t max_x{0.0};
    bool off_track{false};
};

Test test(const std::vector<real_t> &p, uint_t seed, std::ofstream *csv = nullptr)
{
    auto env = make_env();
    auto step = env.reset(seed);
    auto act = actor(p);
    Test t;
    uint_t up_late = 0;
    if (csv)
    {
        *csv << "t,x,theta,force\n";
    }
    while (!step.done())
    {
        const auto force = act(step.observation());
        step = env.step(force);
        t.ret += step.reward();
        const real_t time = env.n_steps() * env.dt();
        const bool up = step.get_extra<bool>("upright");
        if (up && t.swing_up_time < 0.0)
        {
            t.swing_up_time = time;
        }
        if (up && env.n_steps() > MAX_STEPS / 2)
        {
            ++up_late;
        }
        t.max_x = std::max(t.max_x, std::abs(env.dynamics().state()[0]));
        if (csv)
        {
            *csv << time << "," << env.dynamics().state()[0] << "," << env.angle() << ","
                 << env.dynamics().limit(force[0]) << "\n";
        }
    }
    t.off_track = !step.get_extra<bool>("truncated");
    t.up_last_5s = static_cast<real_t>(up_late) / (MAX_STEPS / 2);
    return t;
}

} // namespace

int main(int argc, char **argv)
{
    uint_t generations = 1500;
    std::string csv_path = "swing_up.csv";
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string a = argv[i];
        if (a == "--generations")
        {
            generations = std::strtoul(argv[i + 1], nullptr, 10);
        }
        else if (a == "--csv")
        {
            csv_path = argv[i + 1];
        }
    }
    auto &pool = bitsim::utils::concurrency::default_thread_pool();

    std::printf("1. Learning to swing up, %lu generations of %lu episodes, on %lu threads\n",
                generations, 2 * config().n_pairs, pool.size() + 1);
    std::printf("   the policy has %lu parameters\n\n", NET.n_parameters());
    std::printf("   generation   mean return   best return   return of the mean\n");
    EvolutionStrategy es(config(), initial_parameters());
    std::uint64_t checksum_at_check = 0;
    double time_at_check = 0.0;
    const auto start = std::chrono::steady_clock::now();
    for (uint_t g = 0; g < generations; ++g)
    {
        const auto r = es.step(fitness, pool);
        if (es.generation() == CHECK_AT)
        {
            checksum_at_check = es.checksum();
            time_at_check = seconds_since(start);
        }
        if (g % 250 == 0 || g + 1 == generations)
        {
            std::printf("   %10lu   %11.1f   %11.1f   %18.1f\n", g, r.mean_fitness, r.max_fitness,
                        fitness(es.parameters(), 1000000, g));
        }
    }
    std::printf("   in %.1f s\n\n", seconds_since(start));

    if (generations >= CHECK_AT)
    {
        std::printf("2. The first %lu generations again, on one thread\n", CHECK_AT);
        ThreadPool one(1);
        EvolutionStrategy again(config(), initial_parameters());
        const auto start1 = std::chrono::steady_clock::now();
        // run from inside the pool of one thread, so every episode runs on that thread
        one.submit(
               [&]
               {
                   for (uint_t g = 0; g < CHECK_AT; ++g)
                   {
                       again.step(fitness, one);
                   }
               })
            .get();
        const double t1 = seconds_since(start1);
        std::printf("   %.2f s against %.2f s on %lu threads, a speed-up of %.2f\n", t1,
                    time_at_check, pool.size() + 1, t1 / time_at_check);
        std::printf("   the same parameters, to the last bit: %s\n\n",
                    again.checksum() == checksum_at_check ? "true" : "false");
    }

    std::printf("3. The learned policy, from 20 starts it never trained on\n");
    std::printf("   start   return   up after [s]   up in the last 5 s   furthest from the "
                "centre [m]\n");
    real_t mean_up = 0.0;
    uint_t n_up = 0;
    for (uint_t i = 0; i < 20; ++i)
    {
        std::ofstream csv;
        if (i == 0)
        {
            csv.open(csv_path);
        }
        const auto t = test(es.parameters(), task_seed(2000000, i), i == 0 ? &csv : nullptr);
        mean_up += t.up_last_5s / 20.0;
        n_up += t.up_last_5s > 0.99 ? 1 : 0;
        if (i < 5)
        {
            char up_after[16];
            if (t.swing_up_time < 0.0)
            {
                std::snprintf(up_after, sizeof(up_after), "never");
            }
            else
            {
                std::snprintf(up_after, sizeof(up_after), "%.2f", t.swing_up_time);
            }
            std::printf("   %5lu   %6.1f   %12s   %16.0f %%   %27.2f%s\n", i, t.ret, up_after,
                        100.0 * t.up_last_5s, t.max_x, t.off_track ? "  off the track" : "");
        }
    }
    std::printf("   ...\n   up for the whole of the last 5 s from %lu of 20 starts; up %.0f %% of "
                "that time on average\n",
                n_up, 100.0 * mean_up);
    std::printf("   the first start is in %s\n", csv_path.c_str());
    return 0;
}
