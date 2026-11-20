#ifndef OPTIMIZER_TYPE_H
#define OPTIMIZER_TYPE_H

#include "bitrl/base/bitrl_types.h"
#include <string>

namespace bitrl {
namespace maths{
namespace optim{

///
/// \brief The RenderModeType enum
///
//BETTER_ENUM(OptimzerType, char, INVALID_TYPE=-1, GD=0, SGD, ADAM, RSPROP);

enum class OptimizerType: int_t
{
    INVALID_TYPE=-1,
    GD=0,
    SGD,
    ADAM,
    RSPROP
};

///
/// \brief to_string.  Returns the RenderModeType to its stringrepresentation
/// \param type The RenderModeType to convert
/// \return std::string

//inline
//std::string to_string(OptimizerType type){return type._to_string();}

inline
constexpr bool compare(OptimizerType tp1, OptimizerType tp2){
    return tp1 == tp2;
}

}

}
}
#endif // OPTIMIZER_TYPE_H
