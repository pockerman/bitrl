#ifndef POLICY_BASE_H
#define POLICY_BASE_H

#include "bitrl/rl/policies/policy_type.h"
#include <ostream>

namespace bitrl
{
namespace rl
{
namespace policies
{

///
/// \brief The PolicyBase class
///
class PolicyBase
{
  public:
    ///
    /// \brief ~PolicyBase. Destructor
    ///
    virtual ~PolicyBase() = default;

    ///
    /// \brief type. Returns the policy identifier type
    /// \return
    ///
    PolicyType type() const noexcept { return type_; }

    ///
    /// \brief print
    ///
    virtual std::ostream &print(std::ostream &out) const = 0;

  protected:
    ///
    ///
    ///
    PolicyBase(PolicyType type);

    ///
    /// \brief type_
    ///
    PolicyType type_;
};

inline PolicyBase::PolicyBase(PolicyType type) : type_(type) {}

inline std::ostream &operator<<(std::ostream &out, const PolicyBase &p) { return p.print(out); }

} // namespace policies
} // namespace rl
} // namespace bitrl

#endif // POLICY_BASE_H
