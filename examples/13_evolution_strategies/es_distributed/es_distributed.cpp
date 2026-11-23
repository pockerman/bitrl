// Evolution strategies over several ranks: every rank plays some of the episodes of a
// generation, and the ranks exchange only the returns, since every rank can draw every
// perturbation from the shared seed.
//
// Usage: es_distributed [--generations N] [--ranks N]
//        es_distributed --rank R --ranks N [--generations N] [--host H] [--port P]
//
// Without --rank, the example trains on one rank, and then on N ranks as threads of this
// process, each on one thread. With --rank, it is one rank of N processes; rank 0 runs the
// store at --host:--port.
#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/classic_control/cart_pole_swing_up.h"
#include "bitrl/rl/algorithms/es/evolution_strategy.h"
#include "bitrl/rl/algorithms/es/mlp_policy.h"
#include "bitrl/rl/distributed/communicator.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::classic_control::CartPoleSwingUp;
using bitrl::rl::distributed::Communicator;
using bitrl::rl::distributed::TCPStoreCommunicator;
using bitsim::utils::concurrency::task_seed;
using bitsim::utils::concurrency::ThreadPool;
using namespace bitrl::rl::algos::es;

const uint_t MAX_STEPS = 500;
const MLPPolicy NET({5, 16, 1});

// the task of es_cart_pole_swing_up
real_t fitness(const std::vector<real_t> &p, uint_t g, uint_t k)
{
    CartPoleSwingUp env;
    env.make("v1", {{"max_steps", MAX_STEPS}}, {});
    auto act = [&p](const std::vector<real_t> &o)
    { return NET.act_continuous(p, {o[0] / 2.4, o[1] / 3.0, o[2], o[3], o[4] / 10.0}, 10.0); };
    return episode_return(env, act, task_seed(g, k), MAX_STEPS);
}

std::vector<real_t> initial_parameters()
{
    auto p = NET.initial_parameters(1);
    for (std::size_t j = p.size() - 17; j < p.size(); ++j)
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

// run f on a thread of a pool of one, where parallel_for runs every episode on that thread
template <typename F> void on_one_thread(F &&f)
{
    ThreadPool one(1);
    one.submit([&] { f(one); }).get();
}

struct Run
{
    std::uint64_t checksum{0};
    real_t return_of_mean{0.0};
    double seconds{0.0};
};

// train on this rank; communicator is null for a run without ranks
Run train(Communicator *communicator, uint_t generations)
{
    Run run;
    on_one_thread(
        [&](ThreadPool &pool)
        {
            EvolutionStrategy es(config(), initial_parameters());
            const auto start = std::chrono::steady_clock::now();
            for (uint_t g = 0; g < generations; ++g)
            {
                if (communicator)
                {
                    es.step(fitness, *communicator, pool);
                }
                else
                {
                    es.step(fitness, pool);
                }
            }
            run.seconds = seconds_since(start);
            run.checksum = es.checksum();
            run.return_of_mean = fitness(es.parameters(), 1000000, 0);
        });
    return run;
}

void report_traffic(uint_t ranks)
{
    const uint_t pairs = config().n_pairs;
    const uint_t most = 2 * ((pairs + ranks - 1) / ranks);
    std::printf("   every generation each rank sends the returns of its own pairs, at most %lu "
                "numbers, %lu bytes,\n   and receives the others': %lu returns in all\n",
                most, most * sizeof(real_t), 2 * pairs);
    std::printf("   a gradient of this network would be %lu bytes; of a network of a million "
                "weights, 4 MB:\n   the returns stay %lu bytes, however large the network\n",
                NET.n_parameters() * sizeof(float), most * sizeof(real_t));
}

} // namespace

int main(int argc, char **argv)
{
    uint_t generations = 300, ranks = 4;
    long rank = -1;
    std::string host = "127.0.0.1";
    uint_t port = 29541;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string a = argv[i];
        const char *v = argv[i + 1];
        if (a == "--generations")
            generations = std::strtoul(v, nullptr, 10);
        else if (a == "--ranks")
            ranks = std::strtoul(v, nullptr, 10);
        else if (a == "--rank")
            rank = std::strtol(v, nullptr, 10);
        else if (a == "--host")
            host = v;
        else if (a == "--port")
            port = std::strtoul(v, nullptr, 10);
    }
    if (ranks == 0 || generations == 0)
    {
        std::fprintf(stderr, "need --ranks and --generations of at least 1\n");
        return 1;
    }

    if (rank >= 0)
    {
        TCPStoreCommunicator communicator(host, port, static_cast<uint_t>(rank), ranks);
        const auto run = train(&communicator, generations);
        std::printf("rank %ld of %lu: %lu generations in %.2f s, return of the mean %.1f, "
                    "checksum %lx\n",
                    rank, ranks, generations, run.seconds, run.return_of_mean, run.checksum);
        return 0;
    }

    std::printf("Swinging the pole up: %lu generations of %lu episodes\n\n", generations,
                2 * config().n_pairs);
    std::printf("1. One rank, on one thread\n");
    const auto alone = train(nullptr, generations);
    std::printf("   %.2f s; the return of the mean policy %.1f\n\n", alone.seconds,
                alone.return_of_mean);

    std::printf("2. %lu ranks, each on one thread, every rank %lu of the %lu pairs\n", ranks,
                (config().n_pairs + ranks - 1) / ranks, config().n_pairs);
    std::vector<Run> runs(ranks);
    std::vector<std::thread> threads;
    for (uint_t r = 0; r < ranks; ++r)
    {
        threads.emplace_back(
            [&, r]
            {
                TCPStoreCommunicator communicator(host, port, r, ranks);
                runs[r] = train(&communicator, generations);
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    bool same = true;
    for (const auto &run : runs)
    {
        same = same && run.checksum == alone.checksum;
    }
    std::printf("   %.2f s, a speed-up of %.2f; the return of the mean policy %.1f\n",
                runs[0].seconds, alone.seconds / runs[0].seconds, runs[0].return_of_mean);
    std::printf("   every rank ended with the parameters of the one rank, to the last bit: %s "
                "(checksum %lx)\n",
                same ? "true" : "false", alone.checksum);
    report_traffic(ranks);
    return same ? 0 : 1;
}
