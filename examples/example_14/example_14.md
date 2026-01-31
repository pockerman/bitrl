\page bitrl_example_14 Example 14 Create an environment using Chrono

in this example we will create an environment for reinforcement learning based
on the <a href="https://github.com/projectchrono/chrono">Chrono</a> library.
Specifically, we will create an environment that includes a <a href="https://en.wikipedia.org/wiki/Differential_wheeled_robot">differential drive system</a>.
Note that the model we will create will not be of high fidelity as the purpose of the example is show how
to use Chrono to create reinforcement learning environments.

In order to be able to run this example you need to configure bitrl with Chrono support. You will also
need the <a href="https://irrlicht.sourceforge.io/">Irrlicht</a> library for visualising the robot.

The following image shows an image of the environment we will create

| ![Environment overview](./env.png) |
|:------------------------------------:|


## Create the robot

Below is the class that handles the robot model.

@code{.cpp}
class DiffDriveRobot
{
public:

    void add_to_sys(chrono::ChSystemSMC& sys);
    void build();
    void set_speed(real_t speed);
    void reset();
    bitrl::rb::bitrl_chrono::CHRONO_RobotPose& pose()noexcept{return pose_;}

private:

    std::shared_ptr<chrono::ChBody> chassis_;
    std::pair<std::shared_ptr<chrono::ChBody>, std::shared_ptr<chrono::ChBody>> wheels_;
    std::shared_ptr<chrono::ChBody> caster_wheel_;
    bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose_;

};
@endcode

The chassis of the robot is a simple rectangular plate. It also has three wheels. The model robot we will develop will not consider motors.
However, Chrono allows for high fidelity models is this is needed. Below is the function that build the robot

@code
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
@endcode

The reset function simple resets the robot to its original position

@code
void
DiffDriveRobot::reset()
{
chassis_ -> SetPos(chrono::ChVector3d(0.0, 0.0, 0.22));
wheels_.first  -> SetPos(chrono::ChVector3d(0.0, 0.175, 0.16));
wheels_.second -> SetPos(chrono::ChVector3d(0.0, -0.175, 0.16));
caster_wheel_ -> SetPos(chrono::ChVector3d(0.2, 0.0, 0.16));
}
@endcode

Below are some helper functions for the robot.

@code
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
@endcode

## Create the environment

The environment class inherits from the \ref bitrl::envs::EnvBase "`bitrl::envs::EnvBase`" class. We will need to specify the
time step type and the space type:

@code
constexpr uint_t STATE_SPACE_SIZE = 2;
constexpr uint_t ACTION_SPACE_SIZE = 1;

using bitrl::TimeStepTp;
using bitrl::TimeStep;

//typedef TimeStep<std::vector<real_t> > time_step_type;
typedef TimeStep<chrono::ChVector3d> time_step_type;
typedef bitrl::envs::ContinuousVectorStateContinuousVectorActionEnv<STATE_SPACE_SIZE, STATE_SPACE_SIZE> space_type;
@endcode

@code
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
@endcode

Below are the implementations for reset, step and make

@code
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
@endcode

The simulate function wraps everything together

@code
void DiffDriveRobotEnv::simulate()
{

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
@endcode