#ifndef LOSS_TYPE_H
#define LOSS_TYPE_H

#include "bitrl/extern/enum.h"
#include <string>

namespace cubeai{
namespace utils{

///
/// \brief The RenderModeType enum
///
BETTER_ENUM(LossType, char, INVALID_TYPE=-1, 
                            CROSS_ENTROPY=0, MSE);

///
/// \brief to_string.  Returns the RenderModeType to its stringrepresentation
/// \param type The RenderModeType to convert
/// \return std::string

inline
std::string to_string(LossType type){return type._to_string();}

///
/// \brief compare
/// \param tp1
/// \param tp2
/// \return
///
inline
constexpr bool compare(LossType tp1, LossType tp2){
    return tp1 == tp2;
}


}
}

#endif // LOSS_TYPE_H
