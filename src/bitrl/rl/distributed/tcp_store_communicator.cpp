#include "bitrl/rl/distributed/communicator.h"

#include <torch/csrc/distributed/c10d/TCPStore.hpp>

#include <thread>

namespace bitrl
{
namespace rl::distributed
{

struct TCPStoreCommunicator::Store
{
    c10d::TCPStore store;
    Store(const std::string &host, const c10d::TCPStoreOptions &options) : store(host, options) {}
};

TCPStoreCommunicator::TCPStoreCommunicator(const std::string &host, uint_t port, uint_t rank,
                                           uint_t world_size, std::chrono::milliseconds timeout)
    : rank_(rank), world_size_(world_size)
{
    if (world_size == 0 || rank >= world_size)
    {
        throw std::invalid_argument("TCPStoreCommunicator: rank " + std::to_string(rank) + " of " +
                                    std::to_string(world_size));
    }
    if (port == 0 || port > 65535)
    {
        throw std::invalid_argument("TCPStoreCommunicator: bad port " + std::to_string(port));
    }
    c10d::TCPStoreOptions options;
    options.port = static_cast<std::uint16_t>(port);
    options.isServer = rank == 0;
    options.numWorkers = static_cast<std::size_t>(world_size);
    options.waitWorkers = true;
    options.timeout = timeout;
    store_ = std::make_unique<Store>(host, options);
}

TCPStoreCommunicator::~TCPStoreCommunicator()
{
    try
    {
        auto &store = store_->store;
        store.add("done", 1);
        if (rank_ == 0)
        {
            // keep the store until every rank has read what it needs
            const auto start = std::chrono::steady_clock::now();
            while (store.add("done", 0) < static_cast<int64_t>(world_size_) &&
                   std::chrono::steady_clock::now() - start < std::chrono::seconds(30))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    catch (...)
    {
    }
}

std::string TCPStoreCommunicator::key_(uint_t call, const std::string &who) const
{
    return "c/" + std::to_string(call) + "/" + who;
}

void TCPStoreCommunicator::forget_(uint_t call)
{
    // a rank in call k has read every rank's call k - 1, so nobody still reads call k - 2
    if (call >= 2)
    {
        store_->store.deleteKey(key_(call - 2, std::to_string(rank_)));
        store_->store.deleteKey(key_(call - 2, "b"));
    }
}

std::vector<std::string> TCPStoreCommunicator::all_gather(const std::string &mine)
{
    const uint_t call = n_calls_++;
    auto &store = store_->store;
    forget_(call);
    store.set(key_(call, std::to_string(rank_)), std::vector<uint8_t>(mine.begin(), mine.end()));
    std::vector<std::string> all(world_size_);
    for (uint_t r = 0; r < world_size_; ++r)
    {
        const auto bytes = store.get(key_(call, std::to_string(r)));
        all[r].assign(bytes.begin(), bytes.end());
    }
    return all;
}

std::string TCPStoreCommunicator::broadcast(const std::string &bytes, uint_t root)
{
    if (root >= world_size_)
    {
        throw std::invalid_argument("TCPStoreCommunicator: no rank " + std::to_string(root));
    }
    const uint_t call = n_calls_++;
    auto &store = store_->store;
    forget_(call);
    if (rank_ == root)
    {
        store.set(key_(call, "b"), std::vector<uint8_t>(bytes.begin(), bytes.end()));
    }
    // every rank says it is here, so that a rank in the next call knows this one was read
    store.set(key_(call, std::to_string(rank_)), std::vector<uint8_t>());
    const auto value = store.get(key_(call, "b"));
    for (uint_t r = 0; r < world_size_; ++r)
    {
        store.get(key_(call, std::to_string(r)));
    }
    return std::string(value.begin(), value.end());
}

} // namespace rl::distributed
} // namespace bitrl
