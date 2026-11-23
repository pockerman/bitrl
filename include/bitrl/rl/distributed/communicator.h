#ifndef BITRL_RL_DISTRIBUTED_COMMUNICATOR_H
#define BITRL_RL_DISTRIBUTED_COMMUNICATOR_H

#include "bitrl/base/bitrl_types.h"

#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace bitrl
{
namespace rl::distributed
{

///
/// \brief How the ranks of a distributed training exchange numbers.
///
/// There are world_size() ranks, numbered from 0. Every call is collective: every rank must make
/// the same calls, in the same order, with values of the same size, and each call returns once
/// every rank has made it. all_reduce_sum() adds the values of the ranks in the order of the
/// ranks, so every rank gets exactly the same sum, to the last bit, which keeps the networks of
/// the ranks alike.
///
/// A derived class only moves bytes, with all_gather() and broadcast(); TCPStoreCommunicator
/// moves them through libtorch's TCPStore.
///
class Communicator
{
  public:
    virtual ~Communicator() = default;

    virtual uint_t rank() const noexcept = 0;

    virtual uint_t world_size() const noexcept = 0;

    ///
    /// \brief What every rank gave, in the order of the ranks
    ///
    virtual std::vector<std::string> all_gather(const std::string &mine) = 0;

    ///
    /// \brief What root gave; the other ranks' bytes are ignored
    ///
    virtual std::string broadcast(const std::string &bytes, uint_t root) = 0;

    ///
    /// \brief Replace values with the sum of every rank's values
    ///
    template <typename T> void all_reduce_sum(std::vector<T> &values);

    ///
    /// \brief Replace values with root's
    ///
    template <typename T> void broadcast(std::vector<T> &values, uint_t root);

    ///
    /// \brief Return once every rank has called it
    ///
    void barrier() { all_gather(std::string()); }
};

///
/// \brief A Communicator through c10d::TCPStore, the key-value store that comes with libtorch.
///
/// Rank 0 runs the store on port of host, and every rank, rank 0 included, connects to it, so the
/// ranks can be threads, processes, or machines that reach host. Every value a rank sends goes
/// through the store, so every rank receives world_size() messages per call: plenty for the
/// networks of RL, which have thousands of weights, not for networks of millions, which want
/// a ring all-reduce such as Gloo's or MPI's behind this same interface.
///
/// The store keeps only the last two calls of every rank. On destruction every rank tells the
/// others it is done, and rank 0 keeps the store until all have.
///
class TCPStoreCommunicator final : public Communicator
{
  public:
    ///
    /// \param host where rank 0 runs the store
    /// \param port its port
    /// \param timeout how long a call waits for the other ranks
    ///
    TCPStoreCommunicator(const std::string &host, uint_t port, uint_t rank, uint_t world_size,
                         std::chrono::milliseconds timeout = std::chrono::seconds(60));

    ~TCPStoreCommunicator() override;

    uint_t rank() const noexcept override { return rank_; }

    uint_t world_size() const noexcept override { return world_size_; }

    std::vector<std::string> all_gather(const std::string &mine) override;

    std::string broadcast(const std::string &bytes, uint_t root) override;

  private:
    struct Store;
    std::string key_(uint_t call, const std::string &who) const;
    void forget_(uint_t call);

    std::unique_ptr<Store> store_;
    uint_t rank_;
    uint_t world_size_;
    uint_t n_calls_{0};
};

template <typename T> void Communicator::all_reduce_sum(std::vector<T> &values)
{
    static_assert(std::is_arithmetic_v<T>, "all_reduce_sum needs numbers");
    const std::string mine(reinterpret_cast<const char *>(values.data()),
                           values.size() * sizeof(T));
    const auto all = all_gather(mine);
    std::vector<T> sum(values.size(), T(0));
    for (const auto &bytes : all)
    {
        if (bytes.size() != mine.size())
        {
            throw std::runtime_error("Communicator: the ranks sent values of different sizes");
        }
        std::vector<T> theirs(values.size());
        std::memcpy(theirs.data(), bytes.data(), bytes.size());
        for (std::size_t i = 0; i < sum.size(); ++i)
        {
            sum[i] += theirs[i];
        }
    }
    values = std::move(sum);
}

template <typename T> void Communicator::broadcast(std::vector<T> &values, uint_t root)
{
    static_assert(std::is_arithmetic_v<T>, "broadcast needs numbers");
    const std::string mine(reinterpret_cast<const char *>(values.data()),
                           values.size() * sizeof(T));
    const auto bytes = broadcast(mine, root);
    if (bytes.size() % sizeof(T) != 0)
    {
        throw std::runtime_error("Communicator: broadcast received a partial value");
    }
    values.resize(bytes.size() / sizeof(T));
    std::memcpy(values.data(), bytes.data(), bytes.size());
}

} // namespace rl::distributed
} // namespace bitrl

#endif // BITRL_RL_DISTRIBUTED_COMMUNICATOR_H
