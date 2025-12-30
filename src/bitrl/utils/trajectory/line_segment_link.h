#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/geometry/generic_line.h"
#include "bitrl/utils/trajectory/waypoint.h"

namespace bitrl
{
namespace utils
{
namespace trajectory
{

///
/// \brief class LineSegmentLink. Represents a straight line link
/// between two waypoints.
///
template <int dim, typename NodeData, typename SegmentData>
class LineSegmentLink : private bitrl::utils::geom::GenericLine<dim>
{
  public:
    static const int dimension = dim;
    typedef NodeData node_data_type;
    typedef SegmentData segment_data_type;
    typedef WayPoint<dim, NodeData> w_point_type;

    using bitrl::utils::geom::GenericLine<dim>::get_vertex;
    using bitrl::utils::geom::GenericLine<dim>::get_id;
    using bitrl::utils::geom::GenericLine<dim>::set_id;
    using bitrl::utils::geom::GenericLine<dim>::has_valid_id;
    using bitrl::utils::geom::GenericLine<dim>::length;

    ///
    /// \brief Default ctor
    ///
    LineSegmentLink();

    ///
    /// \brief Constructor
    ///
    LineSegmentLink(const w_point_type &v1, const w_point_type &v2, uint_t id,
                    const segment_data_type &data);

    ///
    /// \brief Constructor
    ///
    LineSegmentLink(const w_point_type &v1, const w_point_type &v2, uint_t id);

    ///
    /// \brief Returns true if the segment is active
    ///
    bool is_active() const { return is_active_; }

    ///
    /// \brief deactive the segment
    ///
    void deactivate() { is_active_ = false; }

    ///
    /// \brief Activate the segment
    ///
    void make_active() { is_active_ = true; }

  private:
    /// \brief list of internal points of
    /// the segment.
    std::vector<w_point_type> internal_points_;

    /// \brief The data asociated with the segmen
    segment_data_type data_;

    /// \brief Flag indicating if the segment is active
    bool is_active_;
};

template <int dim, typename NodeData, typename SegmentData>
LineSegmentLink<dim, NodeData, SegmentData>::LineSegmentLink()
    : bitrl::utils::geom::GenericLine<dim>(w_point_type(), w_point_type(),
                                           bitrl::consts::INVALID_ID),
      internal_points_(), data_(), is_active_(false)
{
}

template <int dim, typename NodeData, typename SegmentData>
LineSegmentLink<dim, NodeData, SegmentData>::LineSegmentLink(
    const typename LineSegmentLink<dim, NodeData, SegmentData>::w_point_type &v1,
    const typename LineSegmentLink<dim, NodeData, SegmentData>::w_point_type &v2, uint_t id,
    const typename LineSegmentLink<dim, NodeData, SegmentData>::segment_data_type &data)
    : bitrl::utils::geom::GenericLine<dim>(v1, v2, id), internal_points_(), data_(data),
      is_active_(true)
{
}

template <int dim, typename NodeData, typename SegmentData>
LineSegmentLink<dim, NodeData, SegmentData>::LineSegmentLink(
    const typename LineSegmentLink<dim, NodeData, SegmentData>::w_point_type &v1,
    const typename LineSegmentLink<dim, NodeData, SegmentData>::w_point_type &v2, uint_t id)
    : LineSegmentLink<dim, NodeData, SegmentData>(
          v1, v2, id, typename LineSegmentLink<dim, NodeData, SegmentData>::segment_data_type())
{
}

} // namespace trajectory
} // namespace utils
} // namespace bitrl