#ifndef ROBOT_H
#define ROBOT_H

#include "bitrl/bitrl_types.h"
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/functions/ChFunctionConst.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>


#include <memory>
namespace example_13
{

using namespace bitrl;

const real_t WHEEL_RADIUS = 0.1;
const real_t CHASSIS_HEIGHT = 0.1;
const real_t WHEEL_WIDTH = 0.05;
const real_t CASTER_RADIUS = 0.05;

auto build_wheel(std::shared_ptr<chrono::ChContactMaterialNSC> material, const chrono::ChVector3d& pos);
auto build_motor(std::shared_ptr<chrono::ChBodyEasyCylinder> wheel,
                 std::shared_ptr<chrono::ChFunctionConst> speed_func,
                 std::shared_ptr<chrono::ChBodyEasyBox> chassis,
                 const chrono::ChVector3d& pos);

class Robot
{
public:

    // constructor
    Robot()=default;

    // add the components of this robot to the system we simulate
    void add_to_sys(chrono::ChSystemNSC& sys) const;

    // build the robot
    void build(std::shared_ptr<chrono::ChContactMaterialNSC> material);

    // set the speeds of the motors
    void set_speed(real_t speed);

    // reset the robot
    void reset();

    // the pose of the robot
    chrono::ChVector3d position()noexcept{return chassis_ -> GetPos();}

private:

    std::shared_ptr<chrono::ChBodyEasyBox> chassis_;
    std::shared_ptr<chrono::ChBodyEasyCylinder> left_wheel_;
    std::shared_ptr<chrono::ChBodyEasyCylinder> right_wheel_;
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> left_wheel_motor_;
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> right_wheel_motor_;
    std::shared_ptr<chrono::ChBodyEasySphere> caster_wheel_;
    std::shared_ptr<chrono::ChLinkLockRevolute> caster_link_joint_;

};
}


#endif //ROBOT_H
