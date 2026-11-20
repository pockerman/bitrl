#ifndef EXAMPLES_CLIFF_WALK_H
#define EXAMPLES_CLIFF_WALK_H

// The cliff walk of Sutton and Barto, example 6.6, written in C++ so that the
// experiments examples need no environment server:
//
//     . . . . . . . . . . . .
//     . . . . . . . . . . . .
//     . . . . . . . . . . . .
//     S C C C C C C C C C C G
//
// The agent starts at S and the episode ends at G. Every step costs 1;
// stepping onto the cliff, C, costs 100 and ends the episode. The actions
// are 0 up, 1 right, 2 down, 3 left, as in Gymnasium's CliffWalking; moving
// off the grid leaves the agent where it is. States are row * 12 + column.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/time_step.h"

#include <string>

namespace examples
{

class CliffWalk
{
  public:
    static constexpr bitrl::uint_t N_ROWS = 4;
    static constexpr bitrl::uint_t N_COLS = 12;
    static constexpr bitrl::uint_t START = (N_ROWS - 1) * N_COLS;
    static constexpr bitrl::uint_t GOAL = N_ROWS * N_COLS - 1;

    typedef bitrl::uint_t state_type;
    typedef bitrl::uint_t action_type;
    typedef bitrl::TimeStep<bitrl::uint_t> time_step_type;

    bitrl::uint_t n_states() const noexcept { return N_ROWS * N_COLS; }
    bitrl::uint_t n_actions() const noexcept { return 4; }

    static bool is_cliff(bitrl::uint_t s)
    {
        return s / N_COLS == N_ROWS - 1 && s % N_COLS > 0 && s % N_COLS < N_COLS - 1;
    }

    static bitrl::uint_t move(bitrl::uint_t s, bitrl::uint_t a)
    {
        bitrl::uint_t row = s / N_COLS;
        bitrl::uint_t col = s % N_COLS;
        if (a == 0 && row > 0)
            row -= 1;
        else if (a == 1 && col < N_COLS - 1)
            col += 1;
        else if (a == 2 && row < N_ROWS - 1)
            row += 1;
        else if (a == 3 && col > 0)
            col -= 1;
        return row * N_COLS + col;
    }

    time_step_type reset()
    {
        state_ = START;
        return time_step_type(bitrl::TimeStepTp::FIRST, 0.0, state_);
    }

    time_step_type step(action_type a)
    {
        state_ = move(state_, a);
        if (is_cliff(state_))
        {
            return time_step_type(bitrl::TimeStepTp::LAST, -100.0, state_);
        }
        if (state_ == GOAL)
        {
            return time_step_type(bitrl::TimeStepTp::LAST, -1.0, state_);
        }
        return time_step_type(bitrl::TimeStepTp::MID, -1.0, state_);
    }

    // The grid with the path of a policy on it, for printing
    template <typename Policy> static std::string draw(const Policy &policy)
    {
        std::string grid(N_ROWS * (N_COLS + 1), '.');
        for (bitrl::uint_t r = 0; r < N_ROWS; ++r)
        {
            grid[r * (N_COLS + 1) + N_COLS] = '\n';
        }
        auto at = [](bitrl::uint_t s) { return (s / N_COLS) * (N_COLS + 1) + s % N_COLS; };
        for (bitrl::uint_t s = START + 1; s < GOAL; ++s)
        {
            grid[at(s)] = 'C';
        }
        bitrl::uint_t s = START;
        for (bitrl::uint_t i = 0; i < 2 * N_ROWS * N_COLS && s != GOAL && !is_cliff(s); ++i)
        {
            grid[at(s)] = '*';
            s = move(s, policy(s));
        }
        grid[at(START)] = 'S';
        grid[at(GOAL)] = 'G';
        return grid;
    }

  private:
    bitrl::uint_t state_{START};
};

} // namespace examples

#endif // EXAMPLES_CLIFF_WALK_H
