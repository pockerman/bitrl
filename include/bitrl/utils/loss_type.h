#ifndef LOSS_TYPE_H
#define LOSS_TYPE_H

#include <string>

namespace bitrl
{
namespace utils
{

///
/// \brief The loss functions a model can be trained with
///
enum class LossType : char
{
    INVALID_TYPE = -1,
    CROSS_ENTROPY = 0,
    MSE
};

///
/// \brief to_string. Returns the name of the given LossType
/// \param type The LossType to convert
/// \return its name, for example "CROSS_ENTROPY"
///
inline std::string to_string(LossType type)
{
    switch (type)
    {
    case LossType::CROSS_ENTROPY:
        return "CROSS_ENTROPY";
    case LossType::MSE:
        return "MSE";
    case LossType::INVALID_TYPE:
        return "INVALID_TYPE";
    }
    return "INVALID_TYPE";
}

///
/// \brief compare. Returns true when the two types are the same
///
inline constexpr bool compare(LossType tp1, LossType tp2) { return tp1 == tp2; }

} // namespace utils
} // namespace bitrl

#endif // LOSS_TYPE_H
