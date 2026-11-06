#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"

#include <chrono/core/ChCoordsys.h>
#include <chrono/core/ChFrame.h>
#include <chrono/core/ChRotation.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITRL_IRRLICHT
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace example_25
{

using bitrl::real_t;
using chrono::ChFrame;
using chrono::ChVector3d;

// Naming convention used throughout this example:
//
//   X_AB is the pose of frame B expressed in frame A
//   p_A  is a point with its coordinates expressed in frame A
//
// W is the world frame, R the robot frame and S the sensor frame. With this
// convention the inner letters cancel when composing: X_WR * X_RS = X_WS and
// X_WS * p_S = p_W

const real_t YAW_90 = bitrl::consts::maths::PI / 2.0;

// The robot sits at (2, 1, 0) in the world and is turned 90 degrees around z
const ChVector3d ROBOT_POSITION(2.0, 1.0, 0.0);

// The sensor is mounted on the left side of the robot and looks to the left
const ChVector3d SENSOR_POSITION(0.0, 0.2, 0.1);

// An obstacle that does not move. It is given in world coordinates
const ChVector3d OBSTACLE_IN_WORLD(0.0, 1.0, 0.1);

std::string to_string(const ChVector3d &v)
{
    // avoid printing tiny round-off errors and -0.000
    const auto clean = [](real_t x) { return std::abs(x) < 5.0e-4 ? 0.0 : x; };

    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "(" << clean(v.x()) << ", " << clean(v.y()) << ", "
        << clean(v.z()) << ")";
    return out.str();
}

// The columns of the rotation matrix are the axes of the frame seen from its parent
void print_frame(const std::string &name, const ChFrame<> &frame)
{
    const auto &R = frame.GetRotMat();
    std::cout << "  " << std::left << std::setw(14) << name
              << " origin=" << to_string(frame.GetPos()) << "  x=" << to_string(R.GetAxisX())
              << "  y=" << to_string(R.GetAxisY()) << "  z=" << to_string(R.GetAxisZ()) << "\n";
}

void print_point(const std::string &name, const ChVector3d &point)
{
    std::cout << "  " << std::left << std::setw(14) << name << " " << to_string(point) << "\n";
}

void print_title(const std::string &title) { std::cout << "\n" << title << "\n"; }

void create_frames()
{
    print_title("1. Creating frames");

    // The default frame is the identity. It coincides with its parent
    ChFrame<> world;
    print_frame("world", world);

    // A position only. The orientation defaults to the identity
    ChFrame<> translated(ROBOT_POSITION);
    print_frame("translated", translated);

    // The same pose written in five different ways
    const ChVector3d z_axis(0.0, 0.0, 1.0);

    // 1. position and quaternion
    ChFrame<> from_quaternion(ROBOT_POSITION, chrono::QuatFromAngleZ(YAW_90));

    // 2. position, angle and a unit axis
    ChFrame<> from_angle_axis(ROBOT_POSITION, YAW_90, z_axis);

    // 3. position and rotation matrix. The arguments are the columns of the
    // matrix: where the x, y and z axes of the new frame point in the parent
    const chrono::ChMatrix33d R(ChVector3d(0.0, 1.0, 0.0), ChVector3d(-1.0, 0.0, 0.0), z_axis);
    ChFrame<> from_matrix(ROBOT_POSITION, R);

    // 4. a chrono::ChCoordsysd which is a position and a quaternion
    ChFrame<> from_coordsys(chrono::ChCoordsysd(ROBOT_POSITION, chrono::QuatFromAngleZ(YAW_90)));

    // 5. start from the identity and set the pose
    ChFrame<> from_setters;
    from_setters.SetPos(ROBOT_POSITION);
    from_setters.SetRot(chrono::QuatFromAngleZ(YAW_90));

    print_frame("robot", from_quaternion);

    const real_t tolerance = 1.0e-12;
    std::cout << "  all five are the same pose: " << std::boolalpha
              << (from_quaternion.Equals(from_angle_axis, tolerance) &&
                  from_quaternion.Equals(from_matrix, tolerance) &&
                  from_quaternion.Equals(from_coordsys, tolerance) &&
                  from_quaternion.Equals(from_setters, tolerance))
              << "\n";
}

void transform_points_and_directions(const ChFrame<> &X_WR)
{
    print_title("2. Points and directions");

    const ChVector3d in_robot(1.0, 0.0, 0.0);

    // A point has a location. Both the rotation and the translation apply
    const ChVector3d point_in_world = X_WR * in_robot;
    print_point("point in R", in_robot);
    print_point("point in W", point_in_world);

    // ...and back. operator/ is the same as TransformPointParentToLocal
    print_point("back in R", X_WR / point_in_world);

    // A direction has no location. Only the rotation applies
    print_point("direction in W", X_WR.TransformDirectionLocalToParent(in_robot));
}

void compose_frames(const ChFrame<> &X_WR, const ChFrame<> &X_RS)
{
    print_title("3. Composing frames");

    // The inner letters cancel: the pose of S in W
    const ChFrame<> X_WS = X_WR * X_RS;
    print_frame("X_RS", X_RS);
    print_frame("X_WS", X_WS);

    // Composition does not commute. This has no physical meaning here
    print_frame("X_RS * X_WR", X_RS * X_WR);
}

void invert_frames(const ChFrame<> &X_WS)
{
    print_title("4. Inverting frames");

    // If X_WS is the pose of the sensor in the world
    // then X_SW is the pose of the world in the sensor
    const ChFrame<> X_SW = X_WS.GetInverse();
    print_frame("X_SW", X_SW);
    print_frame("X_WS * X_SW", X_WS * X_SW);

    // What does the sensor see? The obstacle is straight ahead along its x axis
    const ChVector3d obstacle_in_sensor = X_SW * OBSTACLE_IN_WORLD;
    print_point("obstacle in W", OBSTACLE_IN_WORLD);
    print_point("obstacle in S", obstacle_in_sensor);
    std::cout << "  range = " << obstacle_in_sensor.Length() << " m\n";
}

std::shared_ptr<chrono::ChBody> build_robot(const ChFrame<> &X_WR)
{
    // density, with visual shape, no collision
    auto robot =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(0.6, 0.4, 0.2, 1000.0, true, false);
    robot->SetCoordsys(X_WR.GetCoordsys());
    return robot;
}

void attach_frames_to_body(chrono::ChSystemNSC &system, std::shared_ptr<chrono::ChBody> robot,
                           const ChFrame<> &X_RS)
{
    print_title("5. Frames attached to a moving body");

    // bitrl wraps the body with CHRONO_RobotPose
    bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose(robot);
    const ChVector3d front_of_robot(0.3, 0.0, 0.0);

    // A body is a frame. The pose changes as the simulation advances but
    // X_RS does not since the sensor is rigidly mounted on the robot
    const auto report = [&](const std::string &when)
    {
        const ChFrame<> &X_WR = robot->GetFrameRefToAbs();
        const ChFrame<> X_WS = X_WR * X_RS;

        std::cout << "  " << when << "\n";
        print_frame("X_WR", X_WR);
        print_frame("X_WS", X_WS);
        print_point("front in W", pose.to_world_coords(front_of_robot));
        print_point("obstacle in S", X_WS / OBSTACLE_IN_WORLD);
    };

    report("t = 0.0 s");

    // Turn the robot by 90 degrees over one second
    const real_t step_size = 0.002;
    const real_t end_time = 1.0;
    robot->SetAngVelParent(ChVector3d(0.0, 0.0, YAW_90));

    while (system.GetChTime() < end_time - step_size / 2.0)
    {
        system.DoStepDynamics(step_size);
    }

    report("t = 1.0 s");
}

#ifdef BITRL_IRRLICHT
void render(chrono::ChSystemNSC &system, std::shared_ptr<chrono::ChBody> robot,
            const ChFrame<> &X_RS)
{
    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&system);
    vis.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis.SetWindowSize(1280, 720);
    vis.SetWindowTitle("Example 25: Frames");
    vis.Initialize();
    vis.AddSkyBox();
    vis.AddCamera({5.0, -3.0, 3.0}, ROBOT_POSITION);
    vis.AddTypicalLights();
    vis.BindAll();

    const real_t step_size = 0.002;
    while (vis.Run())
    {
        const ChFrame<> &X_WR = robot->GetFrameRefToAbs();
        const ChFrame<> X_WS = X_WR * X_RS;

        vis.BeginScene();
        vis.Render();

        // x is red, y is green and z is blue
        bitrl::utils::render::irr_draw_world_axes(vis, 2.0);
        chrono::irrlicht::tools::drawCoordsys(&vis, X_WR.GetCoordsys(), 1.0);
        chrono::irrlicht::tools::drawCoordsys(&vis, X_WS.GetCoordsys(), 0.5);

        system.DoStepDynamics(step_size);
        vis.EndScene();
    }
}
#endif

} // namespace example_25

int main(int argc, char **argv)
{
    using namespace example_25;

    // The frames of the lesson. Both of them are constant:
    // X_WR is the initial pose of the robot and X_RS is how the sensor is mounted
    const ChFrame<> X_WR(ROBOT_POSITION, chrono::QuatFromAngleZ(YAW_90));
    const ChFrame<> X_RS(SENSOR_POSITION, chrono::QuatFromAngleZ(YAW_90));

    create_frames();
    transform_points_and_directions(X_WR);
    compose_frames(X_WR, X_RS);
    invert_frames(X_WR * X_RS);

    // no gravity so that the only motion is the rotation we impose
    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    auto robot = build_robot(X_WR);
    system.Add(robot);
    attach_frames_to_body(system, robot, X_RS);

    if (argc > 1 && std::string(argv[1]) == "--render")
    {
#ifdef BITRL_IRRLICHT
        render(system, robot, X_RS);
#else
        std::cerr << "--render needs bitrl configured with BITRL_IRRLICHT=ON" << std::endl;
        return 1;
#endif
    }

    return 0;
}

#else
#include <iostream>
int main()
{
    std::cerr << "You need PROJECTCHRONO configured with "
              << "bitrl in order to run this example "
              << "Reconfigure bitrl and set ENABLE_CHRONO=ON" << std::endl;
    return 1;
}
#endif
