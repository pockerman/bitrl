#ifndef NODE_H
#define NODE_H

#include "bitrl/bitrl_types.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/geometry/mesh/face_element.h"

#include <any>
#include <array>

namespace bitrl
{
namespace utils
{
namespace geom
{

///
/// \brief Wraps the notion of a node. A node
/// is simply a point in dim-space that can hold
/// dofs
///
template <int dim> class Node : public FaceElement<dim, 0>
{

  public:
    ///
    /// \brief Constructor
    ///
    Node();

    ///
    /// \brief Constructor
    ///
    Node(real_t coord, uint_t id, const std::any &data);

    ///
    /// \brief Constructor
    ///
    Node(const std::array<real_t, dim> &coords, uint_t id, const std::any &data);

    ///
    /// \brief Constructor
    ///
    Node(const GeomPoint<dim> &point, uint_t global_id, uint_t pid, const std::any &data);

    ///
    /// \brief Constructor
    ///
    Node(const GeomPoint<dim> &point, uint_t global_id, uint_t pid);
};

} // namespace geom
} // namespace utils
} // namespace bitrl

#endif // NODE_H
