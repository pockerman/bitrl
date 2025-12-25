#include "bitrl/envs/connect2/connect2_env.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/utils/utils.h"

#include <algorithm>
#include <any>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace bitrl
{
namespace envs::connect2
{

const std::string Connect2::name = "Connect2";

Connect2::Connect2()
    : EnvBase<TimeStep<std::vector<uint_t>>,
              DiscreteVectorStateDiscreteActionEnv<53, 0, 4, uint_t>>("Connect2"),
      discount_(1.0), board_()
{
}

Connect2::Connect2(const Connect2 &other)
    : EnvBase<TimeStep<std::vector<uint_t>>,
              DiscreteVectorStateDiscreteActionEnv<53, 0, 4, uint_t>>(other),
      discount_(1.0), board_(other.board_), is_finished_(other.is_finished_)
{
}

void Connect2::make(const std::string & /*version*/,
                    const std::unordered_map<std::string, std::any> &options,
                    const std::unordered_map<std::string, std::any> &reset_options)
{

    board_.resize(4, 0);
    this->set_version_("v1");
    this->make_created_();

    auto idx = utils::uuid4();
    this->set_idx_(idx);
    this->base_type::make("v1", options, reset_options);
    this->make_created_();
}

Connect2::time_step_type Connect2::step(const action_type &action)
{
    return move(player_id_1_, action);
}

Connect2::time_step_type Connect2::reset()
{
    board_ = std::vector<uint_t>(4, 0);
    is_finished_ = false;
    this->get_current_time_step_() =
        Connect2::time_step_type(TimeStepTp::FIRST, 0.0, board_, discount_);
    return this->get_current_time_step_();
}

bool Connect2::is_win(uint_t player) const noexcept
{

    auto player_sum = 0;
    std::for_each(board_.begin(), board_.end(),
                  [&player_sum, player](auto val)
                  {
                      if (val == player)
                          player_sum += 1;
                  });

    return player_sum == win_val_;
}

std::vector<uint_t> Connect2::get_valid_moves() const
{

    std::vector<uint_t> val_moves_;
    val_moves_.reserve(4);

    for (uint_t i = 0; i < board_.size(); ++i)
    {
        if (board_[i] == 0)
        {
            val_moves_.push_back(i);
        }
    }

    return val_moves_;
}

bool Connect2::has_legal_moves() const noexcept
{

    for (auto idx : board_)
    {
        if (idx == 0)
        {
            return true;
        }
    }

    return false;
}

Connect2::time_step_type Connect2::move(const uint_t pid, const action_type &action)
{

    if (pid != 1 && pid != 2)
    {
        throw std::logic_error("Invalid player id: " + std::to_string(pid));
    }

    if (action >= board_.size())
    {
        throw std::logic_error("Invalid action id: " + std::to_string(action));
    }

    if (is_finished_)
    {
        return reset();
    }

    auto valid_move = true;
    if (board_[action] != 0)
    {
        valid_move = false;
    }

    if (valid_move)
    {
        // this position on the board
        // is occupied by the given player
        board_[action] = pid;

        bool won = is_win(pid);
        bool has_moves = has_legal_moves();

        // there may be more moves to make in the game
        // but the player may have won. That's why we look
        // at the won variable first
        auto step_type = TimeStepTp::INVALID_TYPE;
        auto reward = 0.0;
        if (won)
        {
            step_type = TimeStepTp::LAST;
            is_finished_ = true;
            reward = 1.0;
        }
        else if (has_moves)
        {
            // the player has not won the game
            // and there may be more moves
            step_type = TimeStepTp::MID;
            reward = 0.0;
        }
        else
        {
            // the player lost the game
            step_type = TimeStepTp::LAST;
            is_finished_ = true;
            reward = -1.0;
        }

        auto val_moves = get_valid_moves();

        std::unordered_map<std::string, std::any> extra;
        extra["valid_moves"] = std::any(val_moves);
        return Connect2::time_step_type(step_type, reward, board_, discount_, std::move(extra));
    }

    throw std::logic_error("Move: " + std::to_string(action) + " is invalid");
}

} // namespace envs::connect2
} // namespace bitrl