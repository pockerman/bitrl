#include "bitrl/rl/episode_info.h"

namespace bitrl{
namespace rl{

std::ostream&
EpisodeInfo::print(std::ostream& out)const noexcept{

    out<<"Episode index........: "<<episode_index<<std::endl;
    out<<"Episode iterations...: "<<episode_iterations<<std::endl;
    out<<"Episode reward.......: "<<episode_reward<<std::endl;
    out<<"Episode time.........: "<<total_time.count()<<std::endl;
    out<<"Has extra............: "<<std::boolalpha<<!info.empty()<<std::endl;
    return out;
}

}

}
