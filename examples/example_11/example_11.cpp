/*
 * This example shows how to create
 * a waypoint trajectory consisting of straight
 * lines
 * 
 * 
 * 
 */ 

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/utils/trajectory/waypoint_trajectory.h"
#include "bitrl/utils/trajectory/waypoint.h"
#include "bitrl/utils/trajectory/line_segment_link.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/unit_converter.h"

#include <iostream>
#include <string>
#include <map>
#include <any>
#include <array>
#include <vector>

namespace example_11{
	
	using bitrl::real_t;
	using bitrl::uint_t;
	using bitrl::utils::trajectory::WaypointTrajectory;
	using bitrl::utils::trajectory::LineSegmentLink;
	using bitrl::utils::trajectory::WayPoint;
	using bitrl::utils::geom::GeomPoint;
	using bitrl::Null;
	
struct LineSegmentData
{
    /// \brief The maximum velocity
    /// allowed on the edge
    real_t Vmax{0.0};

    /// \brief The orientation of the
    /// segment with respect to the global coordinate
    /// frame. This may also dictate the orientation
    /// that a reference vehicle may have on the segment
    real_t theta{0.0};

    /// \brief The angular velocity on the segment
    real_t w{0.0};

    /// \brief The linear velocity on the segement
    real_t v{0.0};

};

typedef LineSegmentLink<2, Null, LineSegmentData> link_type;
typedef link_type::w_point_type w_point_type;
	
}


int main(){

	using namespace example_11;
	
	// create a trajector of 4 links
	// the waypoints carry not data
	// but the the links 
	// carry objects of type LineSegmentData
	WaypointTrajectory<link_type> trajectory(3);
	
	// start adding the links
	// that form the trajectory
	w_point_type p0(GeomPoint<2>({0.0, 0.0}), static_cast<uint_t>(0));
	w_point_type p1(GeomPoint<2>({1.0, 0.0}), static_cast<uint_t>(1));
	LineSegmentData data{1.0, 0.0, 0.0, 0.95};
	
	link_type l0(p0, p1, static_cast<uint_t>(0), data);
	
	w_point_type p2(GeomPoint<2>({1.0, 0.0}), static_cast<uint_t>(2));
	w_point_type p3(GeomPoint<2>({2.0, 2.0}), static_cast<uint_t>(3));
	data.theta = bitrl::utils::unit_converter::degrees_to_rad(45.0); 
	link_type l1(p2, p3, static_cast<uint_t>(1), data);
	
	w_point_type p4(GeomPoint<2>({2.0, 2.0}), static_cast<uint_t>(4));
	w_point_type p5(GeomPoint<2>({2.0, 3.0}), static_cast<uint_t>(5));
	data.theta = bitrl::utils::unit_converter::degrees_to_rad(90.0); 
	link_type l2(p2, p3, static_cast<uint_t>(2), data);
	
	trajectory[0] = l0;
	trajectory[1] = l1;
	trajectory[2] = l2;

	std::cout<<"Trajectory number of links: "<<trajectory.size()<<std::endl;
	
	for(const auto& link: trajectory){
		std::cout<<link.get_id()<<std::endl;
	}
    return 0;
}
