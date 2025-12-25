#include "bitrl/utils/iterative_algorithm_controller.h"

namespace bitrl
{
namespace utils
{

IterativeAlgorithmResult IterativeAlgorithmController::get_state() const
{

    IterativeAlgorithmResult result;
    result.converged = current_res_ <= exit_tolerance_ ? true : false;
    result.num_iterations = current_iteration_idx_;
    result.tolerance = exit_tolerance_;
    result.residual = current_res_;
    return result;
}

bool IterativeAlgorithmController::continue_iterations()
{

    if (current_iteration_idx_ >= max_iterations_)
    {
        return false;
    }
    else if (current_res_ < exit_tolerance_)
    {

        current_iteration_idx_++;
        return false;
    }

    current_iteration_idx_++;
    return true;
}

void IterativeAlgorithmController::reset(const IterativeAlgorithmController &control)
{
    max_iterations_ = control.max_iterations_;
    exit_tolerance_ = control.exit_tolerance_;
    current_iteration_idx_ = control.current_iteration_idx_;
    current_res_ = control.current_res_;
}

void IterativeAlgorithmController::reset()
{

    current_res_ = std::numeric_limits<real_t>::max();
    current_iteration_idx_ = 0;
}

} // namespace utils
} // namespace bitrl