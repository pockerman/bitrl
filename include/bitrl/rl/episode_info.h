#ifndef EPISODE_INFO_H
#define EPISODE_INFO_H

#include "bitrl/base/bitrl_types.h"
#include "bitsim/bitsim_consts.h"
#include <chrono>
#include <string>
#include <map>
#include <any>
#include <ostream>

namespace bitrl {
namespace rl {

///
/// \brief The EpisodeInfo struct
///
struct EpisodeInfo
{

    uint_t episode_index{bitsim::consts::INVALID_ID};
    uint_t episode_iterations{bitsim::consts::INVALID_ID};
    bool stop_training{false};

    ///
    ///
    ///
    real_t episode_reward{0.0};

    ///
    /// \brief total_time
    ///
    std::chrono::duration<real_t> total_time;

    ///
    /// \brief info
    ///
    std::map<std::string, std::any> info;

    ///
    /// \brief print
    /// \param out
    /// \return
    ///
    std::ostream& print(std::ostream& out)const noexcept;
};

inline
std::ostream& operator<<(std::ostream& out, const EpisodeInfo& info){
    return info.print(out);
}

}

}

#endif // EPISODE_INFO_H
