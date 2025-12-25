#ifndef EPUCK_ROBOT_H
#define EPUCK_ROBOT_H

///
/// \file epuck_robot.h
///

#include "bitrl//bitrl_config.h"

#ifdef BITRL_WEBOTS

#include "rlenvs/rigid_bodies/body_rotation.h"
#include "rlenvs/rigid_bodies/body_translation.h"
#include "rlenvs/rlenvs_types_v2.h"

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

// we need a supervisor as we want to
// be able to reset the simulation
#include <webots/Supervisor.hpp>

#include <memory>
#include <ostream>

namespace rlenvscpp
{
namespace rigid_bodies
{
namespace webots_robots
{

///
/// \brief struct EpuckRobotProperties. Holds the properties
/// of the e-puck robot
///
struct EpuckRobotProperties
{

    ///
    /// \brief Number of distance or proximity sensors available
    ///
    static constexpr uint_t DISTANCE_SENSORS_NUMBER = 8;

    ///
    /// \brief The id for the left position sensor
    ///
    static constexpr uint_t LEFT_POSITION_SENSOR_ID = 0;

    ///
    /// \brief The id for the left position sensor
    ///
    static constexpr uint_t RIGHT_POSITION_SENSOR_ID = 1;

    ///
    /// \brief The id of the left motor
    ///
    static constexpr uint_t LEFT_MOTOR_ID = 0;

    ///
    /// \brief The id of the right motor
    ///
    static constexpr uint_t RIGHT_MOTOR_ID = 0;

    ///
    /// \brief The wheel radius in m
    ///
    static constexpr real_t WHEEL_RADIUS = 0.025;

    ///
    /// \brief The axle length in m
    ///
    static constexpr real_t AXLE_LENGTH = 0.052;

    ///
    /// \brief max fwd speed
    ///
    static constexpr real_t MAX_FWD_SPEED = 0.25;

    ///
    /// \brief max backward speed
    ///
    static constexpr real_t MAX_BWD_SPEED = 0.25;

    ///
    /// \brief Rotation maximum speed rad/s
    ///
    static constexpr real_t MAX_ROTATION_SPEED = 6.28;
};

///
/// \brief struct EpuckOdometry Describes the distance covered
/// by the left and right wheels and the orientation of the robot
///
struct EpuckOdometry
{

    ///
    /// \brief Distance covered by left wheel in m
    ///
    real_t dl;

    ///
    /// \brief Distance covered by right wheel in m
    ///
    real_t dr;

    ///
    /// \brief Orientation of the robot
    ///
    real_t da;

    ///
    /// \brief Constructor
    ///
    EpuckOdometry(real_t dl_, real_t dr_, real_t da_);

    ///
    /// \brief print
    /// \param out
    /// \return
    ///
    std::ostream &print(std::ostream &out) const noexcept;
};

inline EpuckOdometry::EpuckOdometry(real_t dl_, real_t dr_, real_t da_) : dl(dl_), dr(dr_), da(da_)
{
}

inline std::ostream &operator<<(std::ostream &out, const EpuckOdometry &info)
{
    return info.print(out);
}

///
/// \brief class Epuck. Wrapper around webots::Robot class that specifically models
/// an Epuck robot. The robot specification can be found at
/// https://www.cyberbotics.com/doc/guide/epuck?version=cyberbotics:R2019a-rev1
/// Actually this class uses webots::Supervisor instead of webots::Robot
/// see https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_world_save
/// The webots::Supervisor gives more control over the simulation as it
/// allows us to reload the world.
///
class EpuckRobot final : protected EpuckRobotProperties
{
  public:
    ///
    /// \brief name
    ///
    static const std::string name;

    ///
    /// \brief the default time step for simulating the world
    ///
    static constexpr uint_t DEFAULT_SIM_TIME_STEP = 32;

    ///
    /// \brief Expose properties of the robot
    ///
    using EpuckRobotProperties::AXLE_LENGTH;
    using EpuckRobotProperties::DISTANCE_SENSORS_NUMBER;
    using EpuckRobotProperties::LEFT_POSITION_SENSOR_ID;
    using EpuckRobotProperties::MAX_BWD_SPEED;
    using EpuckRobotProperties::MAX_FWD_SPEED;
    using EpuckRobotProperties::MAX_ROTATION_SPEED;
    using EpuckRobotProperties::RIGHT_POSITION_SENSOR_ID;
    using EpuckRobotProperties::WHEEL_RADIUS;

    ///
    /// \brief Create an e-puck robot
    ///
    EpuckRobot();

    ///
    /// \brief Returns the actual webots::Robot pointer
    ///
    std::shared_ptr<webots::Supervisor> get_webots_robot_prt() { return robot_; }

    ///
    /// \brief Activate the motor with the given motor_id
    ///
    void activate_motor(uint_t motor_id, real_t init_velocity);

    ///
    /// \brief Activate the position sensor with the given name
    ///
    void activate_position_sensor(uint_t sensor_id, uint_t time_step);

    ///
    /// \brief Activate the proximity sensor with the given name
    ///
    void activate_proximity_sensor(const std::string &name, uint_t time_step);

    ///
    /// \brief Step in the simulation environment
    ///
    int_t step(uint_t time_step) { return robot_->step(time_step); }

    ///
    /// \brief Begin a time step
    ///
    int_t begin_step(uint_t time_step) { return robot_->stepBegin(time_step); }

    ///
    /// \brief End the last initiated step
    ///
    int_t end_step() { return robot_->stepEnd(); }

    ///
    /// \brief
    ///
    void reset();

    ///
    /// \brief
    ///
    void reload() { robot_->worldReload(); }

    ///
    /// \brief Stop the simulation
    ///
    void stop_simulation() { robot_->simulationQuit(EXIT_SUCCESS); }

    ///
    /// \brief Pause the simulation of this robot
    ///
    void pause_simulation()
    {
        robot_->simulationSetMode(webots::Supervisor::SIMULATION_MODE_PAUSE);
    }

    ///
    /// \brief Compute the odometry value
    ///
    EpuckOdometry compute_odometry() const;

    ///
    /// \brief Get the basic time step used by the robot
    ///
    uint_t get_basic_time_step() const { return robot_->getBasicTimeStep(); }

    ///
    /// \brief Read the distance sensor with the given name
    ///
    real_t get_distance_value_from_sensor(const std::string &sensor_name) const;

    ///
    /// \brief Reads all the distance sensors
    ///
    std::vector<real_t> read_distance_sensors() const;

    ///
    /// \brief Returns the current velocity of
    /// the motor
    ///
    real_t get_motor_velocity(uint_t mid) const { return motors_[mid]->getVelocity(); }

    ///
    /// \brief Get the position of the robot
    ///
    RBTranslation get_position() const;

    ///
    /// \brief Get the position of the robot
    ///
    RBRotation get_rotation() const;

    webots::Motor *get_motor(uint_t m) { return motors_[m]; }
    webots::PositionSensor *get_position_sensor(uint_t m) { return position_sensors_[m]; }

  private:
    /// pointer to the robot to manipulate
    /// see https://cyberbotics.com/doc/reference/robot
    ///
    std::shared_ptr<webots::Supervisor> robot_;

    ///
    /// \brief The position sensors of the robot
    ///
    std::vector<webots::PositionSensor *> position_sensors_;

    ///
    /// \brief The two motors of the robot
    ///
    std::vector<webots::Motor *> motors_;

    ///
    /// \brief Array of the proximity sensors
    ///
    std::vector<webots::DistanceSensor *> proximity_sensors_;
};

} // namespace webots_robots
} // namespace rigid_bodies
} // namespace rlenvscpp

#endif
#endif