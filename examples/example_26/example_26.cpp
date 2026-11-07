#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"

#include <chrono/core/ChFrame.h>
#include <chrono/core/ChMatrix33.h>
#include <chrono/core/ChQuaternion.h>
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/core/ChRotation.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef BITRL_IRRLICHT
#include "bitrl/utils/render/irrlicht_utils.h"
#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace example_26
{

using bitrl::real_t;
using chrono::ChFrame;
using chrono::ChMatrix33d;
using chrono::ChVector3d;

// The same convention as in example_25: R_AB is the orientation of frame B
// expressed in frame A, so that R_WR * R_RS = R_WS and R_WR * v_R = v_W.
// W is the world frame, R the robot frame and S the sensor frame

const real_t DEG = chrono::CH_DEG_TO_RAD;

const ChVector3d X_AXIS(1.0, 0.0, 0.0);
const ChVector3d Y_AXIS(0.0, 1.0, 0.0);
const ChVector3d Z_AXIS(0.0, 0.0, 1.0);

// The same robot and sensor mounting position as in example_25
const ChVector3d ROBOT_POSITION(2.0, 1.0, 0.0);
const ChVector3d SENSOR_POSITION(0.0, 0.2, 0.1);

// The three elementary rotations. Each one leaves its own axis unchanged
ChMatrix33d rot_x(real_t angle)
{
    const real_t c = std::cos(angle);
    const real_t s = std::sin(angle);

    ChMatrix33d R;
    R << 1.0, 0.0, 0.0, //
        0.0, c, -s,     //
        0.0, s, c;
    return R;
}

ChMatrix33d rot_y(real_t angle)
{
    const real_t c = std::cos(angle);
    const real_t s = std::sin(angle);

    ChMatrix33d R;
    R << c, 0.0, s,    //
        0.0, 1.0, 0.0, //
        -s, 0.0, c;
    return R;
}

ChMatrix33d rot_z(real_t angle)
{
    const real_t c = std::cos(angle);
    const real_t s = std::sin(angle);

    ChMatrix33d R;
    R << c, -s, 0.0, //
        s, c, 0.0,   //
        0.0, 0.0, 1.0;
    return R;
}

// A matrix is a rotation when R^T * R = I and det(R) = +1
bool is_rotation(const ChMatrix33d &R, real_t tolerance = 1.0e-9)
{
    const bool orthogonal = (R.transpose() * R - ChMatrix33d(1.0)).norm() < tolerance;
    return orthogonal && std::abs(R.determinant() - 1.0) < tolerance;
}

bool same_matrix(const ChMatrix33d &A, const ChMatrix33d &B, real_t tolerance = 1.0e-9)
{
    return (A - B).norm() < tolerance;
}

struct YawPitchRoll
{
    real_t yaw;
    real_t pitch;
    real_t roll;
};

// Rz(yaw) * Ry(pitch) * Rx(roll)
ChMatrix33d from_yaw_pitch_roll(const YawPitchRoll &angles)
{
    return rot_z(angles.yaw) * rot_y(angles.pitch) * rot_x(angles.roll);
}

// The inverse of from_yaw_pitch_roll. It is not defined when the pitch is +-90 degrees
YawPitchRoll to_yaw_pitch_roll(const ChMatrix33d &R)
{
    return {std::atan2(R(1, 0), R(0, 0)), -std::asin(R(2, 0)), std::atan2(R(2, 1), R(2, 2))};
}

real_t clean(real_t x) { return std::abs(x) < 5.0e-4 ? 0.0 : x; }

std::string to_string(const ChVector3d &v)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "(" << clean(v.x()) << ", " << clean(v.y()) << ", "
        << clean(v.z()) << ")";
    return out.str();
}

void print_matrix(const std::string &name, const ChMatrix33d &R)
{
    std::cout << "  " << name << "\n";
    for (int row = 0; row < 3; ++row)
    {
        std::cout << "    " << std::fixed << std::setprecision(3);
        for (int col = 0; col < 3; ++col)
        {
            std::cout << std::right << std::setw(7) << clean(R(row, col)) << " ";
        }
        std::cout << "\n";
    }
}

void print_vector(const std::string &name, const ChVector3d &v)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << to_string(v) << "\n";
}

void print_number(const std::string &name, real_t value)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << std::fixed
              << std::setprecision(3) << value << "\n";
}

void print_flag(const std::string &name, bool value)
{
    std::cout << "  " << std::left << std::setw(26) << name << " " << std::boolalpha << value
              << "\n";
}

void print_angles(const std::string &name, const YawPitchRoll &angles)
{
    std::cout << "  " << std::left << std::setw(26) << name << " yaw=" << std::fixed
              << std::setprecision(1) << clean(angles.yaw) / DEG
              << " pitch=" << clean(angles.pitch) / DEG << " roll=" << clean(angles.roll) / DEG
              << "\n";
}

void print_title(const std::string &title) { std::cout << "\n" << title << "\n"; }

void elementary_rotations()
{
    print_title("1. Elementary rotations");

    print_matrix("Rx(30)", rot_x(30.0 * DEG));
    print_matrix("Ry(30)", rot_y(30.0 * DEG));
    print_matrix("Rz(30)", rot_z(30.0 * DEG));

    // Chrono builds the same matrices from an angle and an axis
    print_flag("same as Chrono",
               same_matrix(rot_x(30.0 * DEG), ChMatrix33d(30.0 * DEG, X_AXIS)) &&
                   same_matrix(rot_y(30.0 * DEG), ChMatrix33d(30.0 * DEG, Y_AXIS)) &&
                   same_matrix(rot_z(30.0 * DEG), ChMatrix33d(30.0 * DEG, Z_AXIS)));
}

void properties(const ChMatrix33d &R_WR)
{
    print_title("2. What makes a matrix a rotation");

    // The columns are the axes of the robot frame seen from the world
    print_matrix("R_WR", R_WR);
    print_vector("x axis of R", R_WR.GetAxisX());
    print_vector("y axis of R", R_WR.GetAxisY());
    print_vector("z axis of R", R_WR.GetAxisZ());

    print_flag("R_WR is a rotation", is_rotation(R_WR));

    // We do not need to invert the matrix numerically. The transpose is the inverse
    const ChMatrix33d R_RW = R_WR.transpose();
    print_flag("R_RW * R_WR = I", same_matrix(R_RW * R_WR, ChMatrix33d(1.0)));

    // Orthogonal but left-handed: this is a mirror image, not something a body can do
    ChMatrix33d mirror = R_WR;
    mirror.col(2) *= -1.0;
    print_flag("flipped z is a rotation", is_rotation(mirror));

    // Right-handed but it changes lengths
    const ChMatrix33d scaled = 2.0 * R_WR;
    print_flag("scaled is a rotation", is_rotation(scaled));
}

void rotating_vectors(const ChMatrix33d &R_WR)
{
    print_title("3. Rotating vectors");

    // Velocity of a robot that drives forward, expressed in the robot frame
    const ChVector3d v_R(0.5, 0.0, 0.0);
    const ChVector3d v_W = R_WR * v_R;
    print_vector("v in R", v_R);
    print_vector("v in W", v_W);

    // It is the same vector, written in another frame. Its length does not change
    print_number("length in R", v_R.Length());
    print_number("length in W", v_W.Length());

    // Back to the robot frame with the transpose
    print_vector("v back in R", R_WR.transpose() * v_W);
}

ChMatrix33d composing_rotations(const ChMatrix33d &R_WR)
{
    print_title("4. Composing rotations");

    const ChMatrix33d turn = rot_z(90.0 * DEG);
    const ChMatrix33d tilt = rot_y(30.0 * DEG);

    // Turn to the left and then tilt around the y axis of the turned sensor
    const ChMatrix33d turn_then_tilt = turn * tilt;

    // The other order tilts around the y axis of the robot. Afterwards the
    // sensor looks along that same axis so the tilt only rolls it around its beam
    const ChMatrix33d tilt_then_turn = tilt * turn;

    print_matrix("Rz(90) * Ry(30)", turn_then_tilt);
    print_matrix("Ry(30) * Rz(90)", tilt_then_turn);

    print_vector("beam of Rz(90) * Ry(30)", turn_then_tilt * X_AXIS);
    print_vector("beam of Ry(30) * Rz(90)", tilt_then_turn * X_AXIS);

    // The first is what we want. The sensor looks to the left and 30 degrees down
    const ChMatrix33d &R_RS = turn_then_tilt;

    // The inner letters cancel, exactly as they do for frames
    const ChMatrix33d R_WS = R_WR * R_RS;
    const ChVector3d beam_in_world = R_WS * X_AXIS;
    print_flag("R_WS is a rotation", is_rotation(R_WS));
    print_vector("beam in W", beam_in_world);

    // Where does the beam hit the floor z = 0?
    const ChFrame<> X_WS = ChFrame<>(ROBOT_POSITION, R_WR) * ChFrame<>(SENSOR_POSITION, R_RS);
    const real_t range = -X_WS.GetPos().z() / beam_in_world.z();
    print_vector("sensor in W", X_WS.GetPos());
    print_vector("beam hits the floor at", X_WS.GetPos() + beam_in_world * range);

    return R_RS;
}

void yaw_pitch_roll()
{
    print_title("5. Yaw, pitch and roll");

    const YawPitchRoll angles{90.0 * DEG, 10.0 * DEG, 5.0 * DEG};
    const ChMatrix33d R = from_yaw_pitch_roll(angles);
    print_matrix("Rz(90) * Ry(10) * Rx(5)", R);

    // There and back
    print_angles("recovered from R", to_yaw_pitch_roll(R));

    // Chrono offers this with the angles ordered as (pitch, roll, yaw)
    chrono::ChQuaterniond q;
    q.SetFromCardanAnglesZYX(ChVector3d(angles.pitch, angles.roll, angles.yaw));
    print_flag("same as Chrono quaternion", same_matrix(R, ChMatrix33d(q)));

    // Gimbal lock. With the pitch at 90 degrees the yaw and the roll turn around
    // the same axis, so only their difference matters. Different angles, same matrix
    const ChMatrix33d A = from_yaw_pitch_roll({30.0 * DEG, 90.0 * DEG, 0.0});
    const ChMatrix33d B = from_yaw_pitch_roll({50.0 * DEG, 90.0 * DEG, 20.0 * DEG});
    print_flag("gimbal lock: same matrix", same_matrix(A, B));
}

void rotation_of_a_moving_body(const ChMatrix33d &R_WR, const ChMatrix33d &R_RS)
{
    print_title("6. The rotation matrix of a moving body");

    // no gravity so that the only motion is the rotation we impose
    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    auto robot =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(0.6, 0.4, 0.2, 1000.0, true, false);
    robot->SetPos(ROBOT_POSITION);
    robot->SetRot(R_WR);
    system.Add(robot);

    bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose(robot);

    // R_WR changes as the robot moves. R_RS does not, the sensor is rigidly mounted
    const auto report = [&](const std::string &when)
    {
        const ChMatrix33d &current_R_WR = pose.rotation_matrix();
        const ChMatrix33d R_WS = current_R_WR * R_RS;
        std::cout << "  " << when << "\n";
        print_matrix("R_WR", current_R_WR);
        print_flag("R_WR is a rotation", is_rotation(current_R_WR));
        print_vector("beam in W", R_WS * X_AXIS);
    };

    report("t = 0.0 s");

    // Turn the robot by 90 degrees over one second
    const real_t step_size = 0.002;
    const real_t end_time = 1.0;
    robot->SetAngVelParent(ChVector3d(0.0, 0.0, 90.0 * DEG));

    while (system.GetChTime() < end_time - step_size / 2.0)
    {
        system.DoStepDynamics(step_size);
    }

    report("t = 1.0 s");
}

#ifdef BITRL_IRRLICHT
// One of the two rotations of a product
struct Turn
{
    ChVector3d axis;
    real_t angle;
};

// The sensor sits on a mast in this scene. At the real mounting height of 0.1 m
// the beam reaches the floor too close to the robot to see anything
const ChVector3d MAST_TOP(0.0, 0.2, 0.6);
const real_t MAX_BEAM_LENGTH = 2.0;

// One animation: wait, first rotation, wait, second rotation, wait
const real_t ROTATION_TIME = 2.5;
const real_t WAIT_TIME = 1.0;
const real_t CYCLE_TIME = 2.0 * ROTATION_TIME + 3.0 * WAIT_TIME;

// 0 before the interval starts, 1 after it ends
real_t progress(real_t time, real_t start)
{
    return std::clamp((time - start) / ROTATION_TIME, 0.0, 1.0);
}

std::shared_ptr<chrono::ChBody> build_robot_on_mast(const ChVector3d &position)
{
    auto robot =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(0.6, 0.4, 0.2, 1000.0, true, false);
    robot->SetPos(position);
    robot->SetFixed(true);
    robot->GetVisualShape(0)->SetColor(chrono::ChColor(0.55f, 0.70f, 0.90f));

    auto mast = chrono_types::make_shared<chrono::ChVisualShapeBox>(0.03, 0.03, MAST_TOP.z());
    mast->SetColor(chrono::ChColor(0.20f, 0.20f, 0.20f));
    robot->AddVisualShape(mast, ChFrame<>(ChVector3d(MAST_TOP.x(), MAST_TOP.y(), MAST_TOP.z() / 2)));
    return robot;
}

// Draws R_RS = first * second part of the way through the animation, and returns its readout
std::string draw_sensor(chrono::irrlicht::ChVisualSystemIrrlicht &vis,
                        const ChVector3d &robot_position, const std::string &name,
                        const Turn &first, const Turn &second, real_t time)
{
    using namespace bitrl::utils::render;

    const real_t first_done = progress(time, WAIT_TIME);
    const real_t second_done = progress(time, 2.0 * WAIT_TIME + ROTATION_TIME);

    const ChMatrix33d A(first_done * first.angle, first.axis);
    const ChMatrix33d B(second_done * second.angle, second.axis);
    const ChMatrix33d R_RS = A * B;

    const ChVector3d sensor = robot_position + MAST_TOP;
    irr_draw_frame(vis, ChFrame<>(sensor, R_RS), 0.5, "S");

    // The axis we turn around. The second one has been carried along by the first rotation
    const ChVector3d turn_axis = first_done < 1.0 ? first.axis : ChVector3d(A * second.axis);
    irr_draw_line(vis, sensor - turn_axis * 0.8, sensor + turn_axis * 0.8,
                  chrono::ChColor(0.8f, 0.5f, 1.0f), 1.0);

    // The beam stops at the floor if it gets there
    const ChVector3d beam = R_RS * X_AXIS;
    const real_t to_floor = beam.z() < 0.0 ? -sensor.z() / beam.z() : MAX_BEAM_LENGTH;
    const real_t length = std::min(to_floor, MAX_BEAM_LENGTH);
    const ChVector3d end = sensor + beam * length;
    irr_draw_line(vis, sensor, end, chrono::ChColor(1.0f, 0.8f, 0.2f), 3.0);

    if (to_floor < MAX_BEAM_LENGTH)
    {
        chrono::irrlicht::tools::drawCircle(&vis, 0.1, chrono::ChCoordsysd(end),
                                            chrono::ChColor(1.0f, 0.8f, 0.2f));
    }

    irr_draw_label(vis, name, robot_position + ChVector3d(-0.3, -0.3, 0.0));

    std::ostringstream text;
    text << name << "\n\n" << std::fixed << std::setprecision(3);
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            text << std::setw(7) << clean(R_RS(row, col)) << "   ";
        }
        text << "\n";
    }
    text << "\nbeam " << to_string(beam);
    return text.str();
}

irr::gui::IGUIStaticText *add_readout(chrono::irrlicht::ChVisualSystemIrrlicht &vis, int left)
{
    auto *readout = vis.GetGUIEnvironment()->addStaticText(
        L"", irr::core::rect<irr::s32>(left, 10, left + 190, 90), true, true);
    readout->setBackgroundColor(irr::video::SColor(220, 255, 255, 255));
    readout->setDrawBackground(true);
    return readout;
}

// The two orders of section 4, side by side. Both robots face along the world
// x axis, so that R_WR is the identity and what we see is R_RS
void render()
{
    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    const ChVector3d left_robot(-1.5, 0.0, 0.0);
    const ChVector3d right_robot(1.5, 0.0, 0.0);
    system.Add(build_robot_on_mast(left_robot));
    system.Add(build_robot_on_mast(right_robot));

    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&system);
    vis.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis.SetWindowSize(1280, 720);
    vis.SetWindowTitle("Example 26: The order of rotations");
    vis.Initialize();
    vis.AddCamera({6.0, -4.5, 2.6}, {0.0, 0.5, 0.2});
    vis.GetActiveCamera()->setFOV(0.6f);
    vis.AddLightDirectional();
    vis.BindAll();

    auto *left_readout = add_readout(vis, 10);
    auto *right_readout = add_readout(vis, 1080);

    const Turn turn{Z_AXIS, 90.0 * DEG};
    const Turn tilt{Y_AXIS, 30.0 * DEG};

    const real_t step_size = 0.01;
    chrono::ChRealtimeStepTimer real_time;

    while (vis.Run())
    {
        const real_t time = std::fmod(system.GetChTime(), CYCLE_TIME);

        vis.BeginScene();
        vis.Render();

        chrono::irrlicht::tools::drawGrid(&vis, 0.5, 0.5, 12, 8,
                                          chrono::ChCoordsysd(ChVector3d(0.0, 1.0, 0.0)),
                                          chrono::ChColor(0.35f, 0.45f, 0.55f));

        const auto left = draw_sensor(vis, left_robot, "Rz(90) * Ry(30)", turn, tilt, time);
        const auto right = draw_sensor(vis, right_robot, "Ry(30) * Rz(90)", tilt, turn, time);
        left_readout->setText(irr::core::stringw(left.c_str()).c_str());
        right_readout->setText(irr::core::stringw(right.c_str()).c_str());

        system.DoStepDynamics(step_size);
        vis.EndScene();
        real_time.Spin(step_size);
    }
}
#endif

} // namespace example_26

int main(int argc, char **argv)
{
    using namespace example_26;

    // The robot is turned 90 degrees around z, as in example_25
    const ChMatrix33d R_WR = rot_z(90.0 * DEG);

    elementary_rotations();
    properties(R_WR);
    rotating_vectors(R_WR);
    const ChMatrix33d R_RS = composing_rotations(R_WR);
    yaw_pitch_roll();
    rotation_of_a_moving_body(R_WR, R_RS);

    if (argc > 1 && std::string(argv[1]) == "--render")
    {
#ifdef BITRL_IRRLICHT
        render();
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
