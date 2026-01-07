#ifndef ITERATION_COUNTER_H
#define ITERATION_COUNTER_H

#include "bitrl/bitrl_types.h"

namespace bitrl
{
namespace utils
{

///
/// \brief The IterationCounter class
///
class IterationCounter
{
  public:
    ///
    /// \brief IterationCounter
    /// \param max_iterations
    ///
    explicit IterationCounter(uint_t max_itrs) noexcept;

    ///
    /// \brief continue_iterations
    /// \return
    ///
    bool continue_iterations() noexcept;

    ///
    /// \brief current_iteration_index
    /// \return
    ///
    uint_t current_iteration_index() const noexcept { return current_itr_index_; }

    ///
    /// \brief max_iterations
    /// \return
    ///
    uint_t max_iterations() const noexcept { return max_iterations_; }

  private:
    ///
    /// \brief current_itr_index_
    ///
    uint_t current_itr_index_;

    ///
    /// \brief max_iterations_
    ///
    uint_t max_iterations_;
};

inline IterationCounter::IterationCounter(uint_t max_itrs) noexcept
    : current_itr_index_(0), max_iterations_(max_itrs)
{
}

inline bool IterationCounter::continue_iterations() noexcept
{
    if (current_itr_index_ < max_iterations_)
    {
        current_itr_index_++;
        return true;
    }
    return false;
}

} // namespace utils

} // namespace rlenvscpp

#endif // ITERATION_COUNTER_H
