#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include <iostream>
#include <string>

namespace example14
{

using namespace bitrl;
using namespace chrono::irrlicht;

// constants we will be using further below
const uint_t WINDOW_HEIGHT = 800;
const uint_t WINDOW_WIDTH = 1024;
const real_t DT = 0.001;
const real_t SIM_TIME = 5.0;
const std::string WINDOW_TITLE( "Example 14");

void draw_world_axes(chrono::irrlicht::ChVisualSystemIrrlicht& vis,
                     double scale = 1.0) {
    auto* driver = vis.GetVideoDriver();

    // X axis (red)
    driver->draw3DLine(
        {0, 0, 0},
        {static_cast<float>(scale), 0, 0},
        irr::video::SColor(255, 255, 0, 0));

    // Y axis (green)
    driver->draw3DLine(
        {0, 0, 0},
        {0, static_cast<float>(scale), 0},
        irr::video::SColor(255, 0, 255, 0));

    // Z axis (blue)
    driver->draw3DLine(
        {0, 0, 0},
        {0, 0, static_cast<float>(scale)},
        irr::video::SColor(255, 0, 0, 255));
}

auto build_wheel(const std::string &wheel_label, real_t radius,
                 real_t width, const chrono::ChVector3d& pos)
{

    auto material = chrono_types::make_shared<chrono::ChContactMaterialSMC>();
    material->SetYoungModulus(2e7);     // stiffness (important)
    material->SetPoissonRatio(0.3);
    material->SetFriction(0.9f);        // traction
    material->SetRestitution(0.0f);     // no bouncing

    // Optional but recommended
    material->SetAdhesion(0.0);
    material->SetKn(2e5);               // normal stiffness override
    material->SetGn(40.0);              // normal damping
    material->SetKt(2e5);               // tangential stiffness
    material->SetGt(20.0);

    // rotation axis for the wheel
    chrono::ChQuaternion<> q;
    q.SetFromAngleAxis(chrono::CH_PI_2, chrono::ChVector3d(1, 0, 0));

    auto wheel = chrono_types::make_shared<chrono::ChBody>();
    wheel->SetMass(1.0);
    wheel->SetPos(pos);
    wheel->SetName(wheel_label);
    //wheel->SetRot(q);
    wheel->SetRot(chrono::QUNIT);
    wheel->EnableCollision(true);

    chrono::ChQuaternion<> q_cyl;
    q_cyl.SetFromAngleAxis(chrono::CH_PI_2, chrono::VECT_X);

    chrono::ChFrame<> cyl_frame(chrono::VNULL, q_cyl);

    auto cyl_shape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(
    material,
    radius,
    width * 0.5   // half-length
    );

    wheel->AddCollisionShape(cyl_shape);

    wheel->AddVisualShape(
    chrono_types::make_shared<chrono::ChVisualShapeCylinder>(radius, width));
    return wheel;
}


// class to model the robot
class DiffDriveRobot
{
public:

    struct MotorHandle {
        std::shared_ptr<chrono::ChLinkMotorRotationSpeed> motor;
        std::shared_ptr<chrono::ChFunctionConst> speed;
    };

    // add the components of this robot to the systme we simulate
    void add_to_sys(chrono::ChSystemSMC& sys);

    // build the robot
    void build();

    // set the speeds of the motors
    void set_motor_speed(real_t speed);

    // the pose of the robot
    bitrl::rb::bitrl_chrono::CHRONO_RobotPose& pose()noexcept{return pose_;}

private:

    //The chassis of the robot
    std::shared_ptr<chrono::ChBody> chassis_;

    bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose_;

    std::pair<std::shared_ptr<chrono::ChBody>, std::shared_ptr<chrono::ChBody>> wheels_;
    std::shared_ptr<chrono::ChBody> caster_wheel_;
    std::pair<MotorHandle, MotorHandle> motors_;
    MotorHandle caster_motor_;
};

DiffDriveRobot::MotorHandle build_motor(std::shared_ptr<chrono::ChBody> wheel,
                                        std::shared_ptr<chrono::ChBody> chassis,
                                        const std::string &motor_label)
{

    // Build joint frame in ABSOLUTE coordinates
    chrono::ChQuaterniond q;
    q.SetFromAngleAxis(chrono::CH_PI_2, chrono::VECT_X);
    //chrono::ChFrame<> frame(wheel->GetPos(), q);

    // Use the wheel's actual absolute frame
    chrono::ChFrame<> frame = wheel->GetFrameRefToAbs();

    auto motor =  chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    motor->Initialize(
    wheel,
    chassis,
    frame);
    motor->SetName(motor_label);

    // set the speed function (rad/s)
    auto speed_func = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
    motor -> SetSpeedFunction(speed_func);

    auto z = motor->GetFrame2Abs().GetRotMat().GetAxisZ();
    std::cout << "Motor Z axis: " << z << std::endl;
    return {motor, speed_func};
}

void DiffDriveRobot::add_to_sys(chrono::ChSystemSMC& sys)
{
    sys.Add(chassis_);
    sys.Add(wheels_.first);
    sys.Add(wheels_.second);
    sys.AddLink(motors_.first.motor);
    sys.AddLink(motors_.second.motor);
    sys.Add(caster_wheel_);

    auto caster_joint = chrono_types::make_shared<chrono::ChLinkLockSpherical>();
    caster_joint->Initialize(
        caster_wheel_,
        chassis_,
        chrono::ChFrame<>(chrono::ChCoordsys<>(caster_wheel_->GetPos()))
    );
    sys.Add(caster_joint);
}
void DiffDriveRobot::build()
{
    // build the chassis of the robot

    chassis_ = chrono_types::make_shared<chrono::ChBody>();
    chassis_->SetMass(1.0);
    chassis_->SetInertiaXX(chrono::ChVector3d(0.1, 0.1, 0.1));
    chassis_->SetPos(chrono::ChVector3d(0.0, 0.0, 0.0));
    chassis_->SetFixed(false);

    // add visual shape for visualization
    auto vis_shape = chrono_types::make_shared<chrono::ChVisualShapeBox>(
    chrono::ChVector3d(0.4, 0.3, 0.05));
    chassis_ -> AddVisualShape(vis_shape);

    // build the wheels of the robot
    wheels_.first = build_wheel("left_wheel", 0.06, 0.05, chrono::ChVector3d(0.0, 0.175, -0.02));
    wheels_.second = build_wheel("right_wheel", 0.06, 0.05, chrono::ChVector3d(0.0, -0.175, -0.02));
    caster_wheel_ = build_wheel("caster_wheel", 0.06, 0.05, chrono::ChVector3d(-0.2, 0.0, -0.02));

    // build the motors of the robot
    motors_.first = build_motor(wheels_.first, chassis_, "left_wheel");
    motors_.second = build_motor(wheels_.second, chassis_, "right_wheel");
    caster_motor_ = build_motor(caster_wheel_, chassis_, "caster_wheel");

    // we want to tract the chassis pose
    pose_.set_body(chassis_);

}

void DiffDriveRobot::set_motor_speed(real_t speed)
{
    motors_.first.speed -> SetConstant(speed);
    motors_.second.speed -> SetConstant(speed);
    caster_motor_.speed -> SetConstant(speed);
}

void build_system(chrono::ChSystemSMC& sys)
{
    // create material for the ground
    auto material = chrono_types::make_shared<chrono::ChContactMaterialSMC>();

    material->SetYoungModulus(2e7);     // stiffness (important)
    material->SetPoissonRatio(0.3);
    material->SetFriction(0.9f);        // traction
    material->SetRestitution(0.0f);     // no bouncing

    // Optional but recommended
    material->SetAdhesion(0.0);
    material->SetKn(2e5);               // normal stiffness override
    material->SetGn(40.0);              // normal damping
    material->SetKt(2e5);               // tangential stiffness
    material->SetGt(20.0);

    auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        5.0, 5.0, 0.001,
        1000,
        true,   // visual
        true,   // collision
        material
    );
    ground->SetFixed(true);

    // set the gravity acceleration
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0.0, -bitrl::consts::maths::G));
    //sys.Add(ground);
}

void simulate(DiffDriveRobot& robot, chrono::ChSystemSMC& sys)
{

    // create the object that handles the visualization
    chrono::irrlicht::ChVisualSystemIrrlicht visual;
    visual.AttachSystem(&sys);
    visual.SetWindowSize(WINDOW_WIDTH, WINDOW_WIDTH); //WINDOW_HEIGHT);
    visual.SetWindowTitle(WINDOW_TITLE);
    visual.Initialize();
    draw_world_axes(visual);
    visual.AddLogo();
    visual.AddSkyBox();
    visual.AddCamera({0, -2, 1}, {0, 0, 0});
    visual.AddTypicalLights();
    visual.BindAll();

    real_t current_time = 0.0;
    auto& pose = robot.pose();

    robot.set_motor_speed(5.0);
    while (visual.Run())
    {
#ifdef BITRL_LOG
        //BOOST_LOG_TRIVIAL(info)<<"Sim time: " << current_time;
#endif

    // Irrlicht must prepare frame to draw
    visual.BeginScene();

    // .. draw items belonging to Irrlicht scene, if any
    visual.Render();

    // .. draw a grid
    tools::drawGrid(&visual, 0.5, 0.5);
        draw_world_axes(visual, 1.5);
 //    tools::drawCoordinateSystem(
 //     &visual,
 //     chrono::ChCoordsys<>(chrono::VNULL, chrono::QUNIT),
 //     0.5   // axis length
 // );

    // .. draw GUI items belonging to Irrlicht screen, if any
    visual.GetGUIEnvironment()->drawAll();

    //sys.DoStepDynamics(DT);
#ifdef BITRL_LOG
    //BOOST_LOG_TRIVIAL(info)<<"Position: "<<pose.position();
#endif

    // Irrlicht must finish drawing the frame
    visual.EndScene();

    current_time += DT;
}

}

}

int main()
{
    using namespace example14;

    // the system we will be simulating
    chrono::ChSystemSMC sys;

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Building system...: ";
#endif
    build_system(sys);

    DiffDriveRobot robot;

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Building robot...: ";
#endif
    robot.build();
    robot.add_to_sys(sys);

    // simulate
    simulate(robot, sys);

    return 0;
}



#else
#include <iostream>
int main()
{
    std::cerr<<"You need PROJECTCHRONO configured with "
             <<"bitrl in order to run this example "
             <<"Reconfigure bitrl and set ENABLE_CHRONO=ON"<<std::endl;
    return 1;
}

#endif
