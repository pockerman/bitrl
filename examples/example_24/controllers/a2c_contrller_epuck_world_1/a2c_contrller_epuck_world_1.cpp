// File:          a2c_contrller_epuck_world_1.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

#include "cubeai/base/cubeai_types.h"
#include "cubeai/maths/optimization/optimizer_type.h"
#include "cubeai/maths/optimization/pytorch_optimizer_factory.h"
#include "cubeai/maths/statistics/distributions/torch_categorical.h"
#include "cubeai/rl/algorithms/pg/a2c.h"
#include "cubeai/rl/trainers/rl_serial_agent_trainer.h"

#include "rlenvs/envs/webots_envs/epuck_simple_grid_world.h"

namespace rl_example_21
{

using bitrl::maths::stats::TorchCategorical;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::pg::A2CConfig;
using bitrl::rl::algos::pg::A2CSolver;

} // namespace rl_example_21

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
    // create the Robot instance.
    Robot *robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor *motor = robot->getMotor("motorname");
    //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
    //  ds->enable(timeStep);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1)
    {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();

        // Process sensor data here.

        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
