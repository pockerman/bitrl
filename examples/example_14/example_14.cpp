#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/time_step_type.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/utils/bitrl_utils.h"

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChBodyEasy.h>

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include <string>
#include <vector>
#include <unordered_map>

namespace example14
{

using namespace bitrl;
using namespace chrono::irrlicht;

// constants we will be using further below
const uint_t WINDOW_HEIGHT = 800;
const uint_t WINDOW_WIDTH = 1024;
const real_t DT = 0.0001;
const real_t SIM_TIME = 5.0;
const std::string WINDOW_TITLE( "Example 14");

constexpr uint_t STATE_SPACE_SIZE = 2;
constexpr uint_t ACTION_SPACE_SIZE = 1;

using bitrl::TimeStepTp;
using bitrl::TimeStep;

//typedef TimeStep<std::vector<real_t> > time_step_type;
typedef TimeStep<chrono::ChVector3d> time_step_type;
typedef bitrl::envs::ContinuousVectorStateContinuousVectorActionEnv<STATE_SPACE_SIZE, STATE_SPACE_SIZE> space_type;

std::shared_ptr<chrono::ChBody> build_wheel(const std::string &wheel_label, real_t radius,
                                            real_t width, const chrono::ChVector3d& pos);

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

// class to model the robot
class DiffDriveRobot
{
public:

    // add the components of this robot to the system we simulate
    void add_to_sys(chrono::ChSystemSMC& sys);

    // build the robot
    void build();

    // set the speeds of the motors
    void set_speed(real_t speed);

    // reset the robot
    void reset();

    // the pose of the robot
    bitrl::rb::bitrl_chrono::CHRONO_RobotPose& pose()noexcept{return pose_;}

private:

    //The chassis of the robot
    std::shared_ptr<chrono::ChBody> chassis_;
    std::pair<std::shared_ptr<chrono::ChBody>, std::shared_ptr<chrono::ChBody>> wheels_;
    std::shared_ptr<chrono::ChBody> caster_wheel_;
    bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose_;

};

void DiffDriveRobot::build()
{
    // build the chassis of the robot
    chassis_ = chrono_types::make_shared<chrono::ChBody>();
    chassis_->SetMass(1.0);
    chassis_->SetPos(chrono::ChVector3d(0.0, 0.0, 0.22));

    // allow the chassis to move
    chassis_->SetFixed(false);

    // add visual shape for visualization
    auto vis_shape = chrono_types::make_shared<chrono::ChVisualShapeBox>(
    chrono::ChVector3d(0.4, 0.3, 0.05));
    chassis_ -> AddVisualShape(vis_shape);

    // build the wheels of the robot
    wheels_.first = build_wheel("left_wheel", 0.06, 0.05, chrono::ChVector3d(0.0, 0.175, 0.16));
    wheels_.second = build_wheel("right_wheel", 0.06, 0.05, chrono::ChVector3d(0.0, -0.175, 0.16));
    caster_wheel_ = build_wheel("caster_wheel", 0.06, 0.05, chrono::ChVector3d(0.2, 0.0, 0.16));

    // we want to tract the chassis pose
    pose_.set_body(chassis_);

}

void
DiffDriveRobot::reset()
{
    chassis_ -> SetPos(chrono::ChVector3d(0.0, 0.0, 0.22));
    wheels_.first  -> SetPos(chrono::ChVector3d(0.0, 0.175, 0.16));
    wheels_.second -> SetPos(chrono::ChVector3d(0.0, -0.175, 0.16));
    caster_wheel_ -> SetPos(chrono::ChVector3d(0.2, 0.0, 0.16));
}

std::shared_ptr<chrono::ChBody> build_wheel(const std::string &wheel_label, real_t radius,
                                           real_t width, const chrono::ChVector3d& pos)
{

    // rotation axis for the wheel
    chrono::ChQuaternion<> q;
    q.SetFromAngleAxis(chrono::CH_PI_2, chrono::ChVector3d(1, 0, 0));

    auto wheel = chrono_types::make_shared<chrono::ChBody>();
    wheel->SetMass(1.0);
    wheel->SetPos(pos);
    wheel->SetName(wheel_label);
    wheel->SetRot(chrono::QUNIT);
    wheel->SetFixed(false);

    auto visual_shape =  chrono_types::make_shared<chrono::ChVisualShapeCylinder>(radius, width);

    chrono::ChQuaterniond qvis;
    qvis.SetFromAngleAxis(chrono::CH_PI_2, chrono::VECT_X);

    chrono::ChFrame<> vis_frame(chrono::VNULL, qvis);

    wheel->AddVisualShape(visual_shape, vis_frame);
    return wheel;
}

void DiffDriveRobot::add_to_sys(chrono::ChSystemSMC& sys)
{
    sys.Add(chassis_);
    sys.Add(wheels_.first);
    sys.Add(wheels_.second);
    sys.Add(caster_wheel_);

}

void DiffDriveRobot::set_speed(real_t speed)
{
    chassis_ -> SetAngVelLocal(chrono::VNULL);
    chassis_ -> SetAngAccLocal(chrono::VNULL);
    chassis_ -> SetLinVel(chrono::ChVector3d(speed, 0.0, 0.0));
    wheels_.first -> SetLinVel(chrono::ChVector3d(speed, 0.0, 0.0));
    wheels_.second -> SetLinVel(chrono::ChVector3d(speed, 0.0, 0.0));
    caster_wheel_ -> SetLinVel(chrono::ChVector3d(speed, 0.0, 0.0));

}

class DiffDriveRobotEnv final: public bitrl::envs::EnvBase<time_step_type, space_type>
{
public:


    typedef typename space_type::action_type action_type;

    DiffDriveRobotEnv();

    virtual void make(const std::string &version,
                      const std::unordered_map<std::string, std::any> &make_options,
                      const std::unordered_map<std::string, std::any> &reset_options) override;

    virtual void close()override{}

    virtual time_step_type reset()override;
    virtual time_step_type step(const action_type &/*action*/)override;

    void simulate();

private:

    DiffDriveRobot robot_;
    chrono::ChSystemSMC sys_;
    uint_t sim_counter_{0};
    real_t current_time_{0.0};
    void build_system_();

};

DiffDriveRobotEnv::DiffDriveRobotEnv()
    :
bitrl::envs::EnvBase<time_step_type, space_type>(bitrl::utils::uuid4(), "DiffDriveRobotEnv"),
robot_(),
sys_()
{}

void DiffDriveRobotEnv::build_system_()
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
        5.0, 5.0, 0.2,
        1000,
        true,   // visual
        true,   // collision
        material
    );
    ground->SetFixed(true);
    ground->SetPos({0, 0, -0.1});

    // set the gravity acceleration
    sys_.SetGravitationalAcceleration(chrono::ChVector3d(0, 0.0, -bitrl::consts::maths::G));
    sys_.Add(ground);
}

void
DiffDriveRobotEnv::make(const std::string &version,
                        const std::unordered_map<std::string, std::any> &make_options,
                        const std::unordered_map<std::string, std::any> &reset_options)
{

    robot_.build();
    build_system_();

    robot_.add_to_sys(sys_);

    this -> set_make_options_(make_options);
    this -> set_reset_options_(reset_options);
    this -> set_version_(version);
    this -> make_created_();

}

time_step_type
DiffDriveRobotEnv::reset()
{
    sim_counter_ = 1;
    current_time_ = 0.0;
    robot_.reset();
    robot_.set_speed(15.0);

    auto robot_position  =robot_.pose().position();
    return time_step_type(TimeStepTp::FIRST, 0.0, robot_position);
}

time_step_type
DiffDriveRobotEnv::step(const action_type &/*action*/)
{

    if (sim_counter_ % 100 == 0)
    {
#ifdef BITRL_LOG
        BOOST_LOG_TRIVIAL(info)<<"Reset simulation: ";
#endif
        return reset();
    }

    sys_.DoStepDynamics(DT);
    auto robot_position  =robot_.pose().position();

    sim_counter_++;
    current_time_ += DT;
    return time_step_type(TimeStepTp::MID, 1.0, robot_position);
}



void DiffDriveRobotEnv::simulate()
{
    // create the object that handles the visualization
    chrono::irrlicht::ChVisualSystemIrrlicht visual;
    visual.AttachSystem(&sys_);
    visual.SetWindowSize(WINDOW_WIDTH, WINDOW_WIDTH); //WINDOW_HEIGHT);
    visual.SetWindowTitle(WINDOW_TITLE);
    visual.Initialize();
    draw_world_axes(visual);
    visual.AddLogo();
    visual.AddSkyBox();
    visual.AddCamera({0, -2, 1}, {0, 0, 0});
    visual.AddTypicalLights();
    visual.BindAll();

    // we need this
    while (visual.Run())
    {

    // Irrlicht must prepare frame to draw
    visual.BeginScene();

    // .. draw items belonging to Irrlicht scene, if any
    visual.Render();

    // .. draw a grid
    tools::drawGrid(&visual, 0.5, 0.5);
    draw_world_axes(visual, 1.5);

    auto time_step = step( action_type());
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"At time: "<<current_time_<<" position: "<<time_step.observation();
#endif

    // Irrlicht must finish drawing the frame
    visual.EndScene();
    }

}

}

int main()
{
    using namespace example14;


#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Building system...: ";
#endif


    DiffDriveRobotEnv env;
    env.make("v1", std::unordered_map<std::string, std::any>(),
    std::unordered_map<std::string, std::any>());
    env.reset();

    env.simulate();
    env.close();
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
