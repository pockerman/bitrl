#ifndef TD_ALGO_BASE_H
#define TD_ALGO_BASE_H


#include "bitrl/rl/algorithms/rl_algorithm_base.h"
#include "bitrl/rl/worlds/envs_concepts.h"

namespace bitrl {
namespace rl::algos::td
{


    ///
///\brief The TDAlgoBase class. Base class
/// for deriving TD algorithms
///
    template<typename EnvType>
    class TDAlgoBase: public RLSolverBase<EnvType>
    {
    public:

        ///
    /// \brief env_t
    ///
        typedef EnvType env_type;

        ///
    /// \brief action_t
    ///
        typedef typename env_type::action_type action_type;

        ///
    /// \brief state_t
    ///
        typedef typename env_type::state_type state_type;

        ///
    /// \brief Destructor
    ///
        virtual ~TDAlgoBase() = default;


    protected:

        ///
    /// \brief DPAlgoBase
    /// \param name
    ///
        TDAlgoBase()=default;
    };


}
}

#endif // TD_ALGO_BASE_H
