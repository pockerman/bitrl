#ifndef EPSILON_DECAY_OPTIONS_H
#define EPSILON_DECAY_OPTIONS_H

#include <string>

namespace bitrl
{
namespace rl
{

enum class EpsilonDecayOptionType : int
{
    INVALID_TYPE = 0,
    NONE,
    EXPONENTIAL,
    INVERSE_STEP,
    CONSTANT_RATE
};

///
/// \brief to_string.  Returns the RenderModeType to its stringrepresentation
/// \param type The RenderModeType to convert
/// \return std::string

// inline
// std::string to_string(EpsilonDecayOptionType type){return type._to_string();}

} // namespace rl

} // namespace bitrl

#endif // EPSILON_DECAY_OPTIONS_H
