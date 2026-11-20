#ifndef RL_ALGO_INPUT_H
#define RL_ALGO_INPUT_H

#include "bitrl/base/cubeai_types.h"

namespace bitrl{
namespace rl{
namespace algos{

///
/// \brief The RLAlgoConfig struct
///
struct RLAlgoConfig
{
    uint_t n_episodes;
    uint_t n_itrs_per_episode;
    uint_t render_env_frequency;
    uint_t output_msg_frequency;
    real_t tolerance;
    bool render_environment{false};
    bool render_episode{false};

};
}
}
}

#endif // RL_ALGO_INPUT_H
