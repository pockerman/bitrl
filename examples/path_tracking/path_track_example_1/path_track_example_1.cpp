/*#include "cubeai/base/cubeai_types.h"
#include "cubeai/control/pure_pursuit_path_tracker.h"
#include "cubeai/geom_primitives/waypoint_path.h"
#include "cubeai/robots/diff_drive_robot.h"


#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <tuple>
#include <utility>

namespace path_track_example_1
{

using cubeai::real_t;
using cubeai::uint_t;
using cubeai::DynMat;
using cubeai::DynVec;
using cubeai::geom_primitives::WaypointPath;
using cubeai::geom_primitives::WayPoint;
using cubeai::geom_primitives::GeomPoint;
using cubeai::geom_primitives::LineSegmentData;
using cubeai::robots::DiffDriveRobot;
using cubeai::control::PurePursuit2DPathTracker;

const uint_t N_STEPS = 10;
const real_t DT = 0.01;
const real_t GOAL_RADIUS = 0.5;
const real_t LOOK_AHEAD_DIST = 1.5;

struct WaypointData
{
    real_t v;
    real_t w;
};

typedef WaypointPath<2, WaypointData, LineSegmentData> path_type;



path_type build_path(){

    path_type path;

    // a path with 10 points
    path.reserve_nodes(10);

    path.add_node({0.0, 0.0});
    path.add_node({1.0, 1.0});
    path.add_node({2.0, 1.0});
    path.add_node({2.5, 1.5});
    path.add_node({3.5, 2.5});
    path.add_node({4.0, 2.5});
    path.add_node({5.0, 2.5});
    path.add_node({6.0, 1.5});
    path.add_node({7.0, 0.5});
    path.add_node({8.0, 0.0});
    return path;
}

class Agent
{

public:

    Agent(const GeomPoint<2>& goal, real_t goal_radius, real_t lookahead_dist);

    void step(const path_type& path);


private:


   DiffDriveRobot robot_;
   PurePursuit2DPathTracker path_tracker_;
};

Agent::Agent(const GeomPoint<2>& goal, real_t goal_radius, real_t lookahead_dist)
     :
       robot_(),
       path_tracker_(goal, lookahead_dist, goal_radius)
 {}

void
Agent::step(const path_type& path){

    // find the velocity commands
    auto current_pose = robot_.get_pose();
    auto vel_cmds = path_tracker_.execute(path, current_pose);

    // execute the commands

}

}*/

int main()
{

    /*using namespace path_track_example_1;

    // build the path
    auto path = build_path();

    Agent agent(path.back(), GOAL_RADIUS, LOOK_AHEAD_DIST);

    for(uint_t itr=0; itr<N_STEPS; ++itr){

        agent.step(path);
    }*/

    return 0;
}
