#include "bitrl/envs/grid_world/grid_world_env.h"

#include <iostream>
#include <random>
#include <set>
#include <string>
#include <utility>

namespace bitrl::envs::grid_world
{



GridWorldInitType from_string(const std::string &gw_init_type)
{

    if (gw_init_type == "STATIC")
    {
        return bitrl::envs::grid_world::GridWorldInitType::STATIC;
    }

    if (gw_init_type == "RANDOM")
    {
        return bitrl::envs::grid_world::GridWorldInitType::RANDOM;
    }

    if (gw_init_type == "PLAYER")
    {
        return bitrl::envs::grid_world::GridWorldInitType::PLAYER;
    }

    return bitrl::envs::grid_world::GridWorldInitType::INVALID_TYPE;
}

namespace detail
{

struct not_equal
{
    bool operator()(board_position a, board_position b) const
    {
        return a != b || a.first != b.first || a.second != b.second;
    }
};

bool validate_board(const board &b)
{

    auto valid = true;

    const auto player_itr = b.components.find(board_component_type::PLAYER);
    if (player_itr == b.components.end())
    {
        throw std::logic_error("PLAYER is missing from the board");
    }

    const auto goal_itr = b.components.find(board_component_type::GOAL);
    if (goal_itr == b.components.end())
    {
        throw std::logic_error("GOAL is missing from the board");
    }

    const auto pit_itr = b.components.find(board_component_type::PIT);
    if (pit_itr == b.components.end())
    {
        throw std::logic_error("PIT is missing from the board");
    }

    auto wall_itr = b.components.find(board_component_type::WALL);
    if (wall_itr == b.components.end())
    {
        throw std::logic_error("WALL is missing from the board");
    }

    /*static uint_t corners[][] = {{0,0},
                                                             {0, board.board_size},
                                                             {board.board_size, 0},
    {board.board_size,board.board_size}};*/

    auto player_pos = player_itr->second.pos;
    auto goal_pos = goal_itr->second.pos;
    auto pit_pos = pit_itr->second.pos;
    auto wall_pos = wall_itr->second.pos;

    // make sure that the positions are distinct
    std::set<board_position, not_equal> positions;
    positions.insert(player_pos);
    positions.insert(goal_pos);
    positions.insert(pit_pos);
    positions.insert(wall_pos);

    if (positions.size() != 4)
    {
        return false;
    }

    static std::set<std::pair<int, int>, not_equal> corners;
    corners.insert({0, 0});
    corners.insert({0, b.board_size});
    corners.insert({b.board_size, 0});
    corners.insert({b.board_size, b.board_size});

    if (corners.contains(player_pos) || corners.contains(goal_pos))
    {

        // validate the moves for the player
        auto pos = std::vector<std::pair<int, int>>({{0, 1}, {1, 0}, {-1, 0}, {0, -1}});

        std::set<int> player_valid_move;
        std::set<int> goal_valid_move;
        for (auto x : pos)
        {

            auto valid_p = b.validate_move(board_component_type::PLAYER, x);
            player_valid_move.insert(static_cast<int>(valid_p));
            auto valid_g = b.validate_move(board_component_type::GOAL, x);
            goal_valid_move.insert(static_cast<int>(valid_g));
        }

        if (!player_valid_move.contains(0) || !goal_valid_move.contains(0))
        {
            return false;
        }
    }

    return true;
}

bool operator==(const board_position &p1, const board_position &p2)
{

    if (p1.first == p2.first && p1.second == p2.second)
        return true;

    return false;
}

bool operator!=(const board_position &p1, const board_position &p2) { return !(p1 == p2); }

board_position operator+(const board_position &p1, const board_position &p2)
{
    return {p1.first + p2.first, p1.second + p2.second};
}

int_t max(const board_position &p) { return std::max(p.first, p.second); }

int_t min(const board_position &p) { return std::min(p.first, p.second); }

void board::close()
{

    components.clear();
    masks.clear();
}

board_state_type board::init_board(uint_t board_s, GridWorldInitType init_type)
{

    // TODO: make sure board_s  != 0
    board_size = board_s;

    // add the pieces on the board
    components[board_component_type::PLAYER] = board_piece("Player", std::make_pair(0, 0));
    components[board_component_type::GOAL] = board_piece("Goal", std::make_pair(1, 0));
    components[board_component_type::PIT] = board_piece("Pit", std::make_pair(2, 0));
    components[board_component_type::WALL] = board_piece("Wall", std::make_pair(3, 0));

    switch (init_type)
    {

    case GridWorldInitType::STATIC:
    {
        build_static_mode();
        break;
    }
    case GridWorldInitType::RANDOM:
    {
        build_random_mode();
        break;
    }
    case GridWorldInitType::PLAYER:
    {
        build_player_mode(seed);
        break;
    }
#ifdef BITRL_DEBUG
    default:
    {
        assert(false && "Invalid initialization mode");
    }
#endif
    }

    if (const auto is_valid_board = validate_board(*this); !is_valid_board)
    {
        throw std::logic_error("The board is invalid");
    }

    is_board_init = true;
    return get_state();
}

board_state_type board::step(GridWorldActionType action)
{

    //  check if the resulting move is valid and
    // move the Player there
    switch (action)
    {
    case GridWorldActionType::UP:
    {
        check_and_move(-1, 0);
        break;
    }
    case GridWorldActionType::DOWN:
    {
        // down
        check_and_move(1, 0);
        break;
    }
    case GridWorldActionType::LEFT:
    {
        // left
        check_and_move(0, -1);
        break;
    }
    case GridWorldActionType::RIGHT:
    {
        // right
        check_and_move(0, 1);
        break;
    }
#ifdef BITRL_DEBUG
    default:
    {
        assert(false && "Invalid move");
    }
#endif
    }

    return get_state();
}

real_t board::get_reward() const
{

    const auto player_pos = components.find(board_component_type::PLAYER)->second.pos;
    const auto pit_pos = components.find(board_component_type::PIT)->second.pos;
    const auto goal_pos = components.find(board_component_type::GOAL)->second.pos;

    // check where the player is
    // if the player is at the PIT position
    // then the game is lost
    if (player_pos == pit_pos)
    {
        return -10.;
    }

    if (player_pos == goal_pos)
    {
        return 10.0;
    }

    return -1.0;
}

board_state_type board::get_state() const
{

    // initialize the data struct the
    // represents the state
    auto num_pieces = components.size() + masks.size();
    board_state_type array_board(num_pieces);

    for (uint_t i = 0; i < num_pieces; ++i)
    {
        array_board[i].resize(board_size);

        for (uint_t j = 0; j < board_size; ++j)
        {
            array_board[i][j].resize(board_size);

            for (uint_t k = 0; k < board_size; ++k)
            {
                array_board[i][j][k] = 0;
            }
        }
    }

    auto layer = 0;

    // check if we can move the piece
    auto comp_begin = components.begin();
    auto comp_end = components.end();

    for (; comp_begin != comp_end; ++comp_begin)
    {

        auto position = comp_begin->second.pos;
        array_board[layer][position.first][position.second] = 1;
        layer += 1;
    }

    return array_board;
}

void board::move_piece(board_component_type piece, board_position pos)
{

    auto move = true;

    // check if we can move the piece
    // auto mask_begin = masks.begin();
    // auto mask_end = masks.end();

    /*for(; mask_begin != mask_end; ++mask_begin){
        position =
    }*/

    //        for _, mask in self.masks.items():
    //            if pos in zip_positions2d(mask.get_positions()):
    //                move = False
    if (move)
    {
        components[piece].pos = pos;
    }
}

board_move_type board::validate_move(board_component_type piece, board_position pos) const
{

    // 0 is valid
    // auto outcome = 0 #0 is valid, 1 invalid, 2 lost game
    auto outcome = board_move_type::VALID;

    const auto piece_pos = components.find(piece)->second.pos;
    const auto pit_pos = components.find(board_component_type::PIT)->second.pos;
    const auto wall_pos = components.find(board_component_type::WALL)->second.pos;
    const auto new_pos = piece_pos + pos;

    //  check if the resulting move is valid and
    // move the Player there

    if (new_pos == wall_pos)
    {
        // 1 //block move, player can't move to wall
        outcome = board_move_type::INVALID;
    }
    else if (max(new_pos) > (board_size - 1))
    {
        // #if outside bounds of board
        outcome = board_move_type::INVALID;
    }
    else if (min(new_pos) < 0)
    {
        // #if outside bounds
        outcome = board_move_type::INVALID;
    }
    else if (new_pos == pit_pos)
    {
        outcome = board_move_type::LOST_GAME;
    }

    return outcome;
}

void board::check_and_move(int_t row, int_t col)
{

    // create a position object from the
    // given row, col entries
    const auto position = std::make_pair(row, col);

    // if the outcome is we lost the game or
    // a valid move we calculate the new position
    // and move the Player there
    if (const auto move_type = validate_move(board_component_type::PLAYER, position);
        move_type == board_move_type::VALID || move_type == board_move_type::LOST_GAME)
    {

        const auto new_pos = components[board_component_type::PLAYER].pos + position;
        move_piece(board_component_type::PLAYER, new_pos);
    }
}

void board::build_static_mode()
{

    // Row, Column
    components[board_component_type::PLAYER].pos = std::make_pair(0, 3);
    components[board_component_type::GOAL].pos = std::make_pair(0, 0);
    components[board_component_type::PIT].pos = std::make_pair(0, 1);
    components[board_component_type::WALL].pos = std::make_pair(1, 1);
}

void board::build_random_mode()
{

    // generate random index
    std::mt19937 generator(seed);

    std::vector<real_t> weights(board_size, 1.0 / static_cast<real_t>(board_size));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());

    components[board_component_type::PLAYER].pos =
        std::make_pair(distribution(generator), distribution(generator));
    components[board_component_type::GOAL].pos =
        std::make_pair(distribution(generator), distribution(generator));
    components[board_component_type::PIT].pos =
        std::make_pair(distribution(generator), distribution(generator));
    components[board_component_type::WALL].pos =
        std::make_pair(distribution(generator), distribution(generator));

    constexpr uint_t n_retries = 20;
    uint_t current_n_retries = 0;
    while (!validate_board(*this) && current_n_retries++ < n_retries)
    {

        // place player randomly on the grid
        components[board_component_type::PLAYER].pos =
            std::make_pair(distribution(generator), distribution(generator));
    }
}

void board::build_player_mode(const uint_t seed)
{

    // height x width x depth (number of pieces)
    build_static_mode();

    // generate random index
    std::mt19937 generator(seed);
    std::vector<real_t> weights(board_size, 1.0 / static_cast<real_t>(board_size));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());

    auto x = distribution(generator);
    auto y = distribution(generator);

    // place player randmly on the grid
    components[board_component_type::PLAYER].pos = std::pair<int, int>(x, y);

    constexpr uint_t n_retries = 20;
    uint_t current_n_retries = 0;
    while (!validate_board(*this) && current_n_retries++ < n_retries)
    {

        x = distribution(generator);
        y = distribution(generator);

        // place player randmly on the grid
        components[board_component_type::PLAYER].pos = std::pair<int, int>(x, y);
    }
}

} // namespace detail

} // namespace bitrl::envs::grid_world
