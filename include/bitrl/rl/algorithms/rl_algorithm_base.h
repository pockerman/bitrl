#ifndef RL_ALGORITHM_BASE_H
#define RL_ALGORITHM_BASE_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/episode_info.h"


#include <boost/noncopyable.hpp>

namespace bitrl{
namespace rl::algos
{

    ///
/// \brief RLAlgoBase. Base class for RL algorithms
///
    template<typename EnvType>
    class RLSolverBase: private boost::noncopyable{

    public:

        typedef EnvType env_type;

        ///
    /// \brief Destructor
    ///
        virtual ~RLSolverBase()=default;

        ///
    /// \brief actions_before_training_begins. Execute any actions the
    /// algorithm needs before starting the iterations
    ///
        virtual void actions_before_training_begins(env_type&) = 0;

        ///
    /// \brief actions_after_training_ends. Actions to execute after
    /// the training iterations have finisehd
    ///
        virtual void actions_after_training_ends(env_type&) = 0;

        ///
    /// \brief actions_before_training_episode
    ///
        virtual void actions_before_episode_begins(env_type&, uint_t /*episode_idx*/){}

        ///
    /// \brief actions_after_training_episode
    ///
        virtual void actions_after_episode_ends(env_type&, uint_t /*episode_idx*/, const EpisodeInfo& /*einfo*/){}

        ///
    /// \brief on_episode Do one on_episode of the algorithm
    ///
        virtual EpisodeInfo on_training_episode(env_type&, uint_t /*episode_idx*/) = 0;

    protected:

        ///
    /// \brief Constructor
    ///
        RLSolverBase()=default;

    };

}
}

#endif // RL_ALGORITHM_BASE_H
