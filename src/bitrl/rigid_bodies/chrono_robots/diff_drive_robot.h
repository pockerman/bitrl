#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"
#include "bitrl/sensors/sensor_manager.h"
#include "bitrl/sensors/sensor_base.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/physics/ChSystemNSC.h>

#include <memory>
#include <filesystem>

namespace bitrl
{
namespace rb::bitrl_chrono
{

/**
 * @class CHRONO_TurtleRobot
 * @ingroup rb_chrono
 * @brief Class for modelling a differential-drive robot using Chrono.
 * in fact the implementation of this class is taken from the
 * Chrono::TurtleBot.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot
{
public:

    ///
    /// Helper class to assemble some robot data
    ///
    struct RobotData
    {
        real_t max_speed;
        real_t wheel_radius;
        real_t wheel_width;
        real_t axle_length;
        real_t density;
        real_t caster_wheel_radius;

        template<typename Archive>
        void read(const Archive& arch);
    };

    ///
    /// Class to manage the sensors attached on this
    ///
    class RobotSensorManager
    {

    public:

        ///
        /// Constructor
        ///
        RobotSensorManager();

        /// Load the sensors of the robot from the archive
        /// @tparam Archive
        /// @param arch
        template <typename Archive>
        void read(const Archive& arch, std::shared_ptr<chrono::ChBody> body);

        ///
        /// @return The number of registered senosrs
        uint_t n_sensors()const noexcept{return sensor_manager_.n_sensors();}

        ///
        /// @brief Add a new sensor to manage
        /// @param sensor
        ///
        void add(std::shared_ptr<sensors::SensorBase> sensor){sensor_manager_.add(sensor);}

    private:

        bitrl::sensors::SensorManager sensor_manager_;

    };

    ///
    /// @brief Constructor
    ///
    CHRONO_DiffDriveRobot()=default;

    /// @brief Load the robot details form the specified filename
    ///
    void load(const std::filesystem::path& filename);

    /// Brief return the number of wheel motors this robot has
    /// @return
    ///
    uint_t n_wheel_motors()const noexcept{return 2;}

    ///
    /// Set the speed of the robot.
    ///  This function sets the x-component of the velocity vector
    ///  of the chassis
    /// @param speed
    void set_speed(real_t speed);

    ///
    /// Set the speed of the m-th motor
    /// m=0 => left motor
    /// m=1 => right motor
    void set_motor_speed(uint_t m, real_t speed);

    ///
    /// Set both motors to the same speed
    /// @param speed
    ///
    void set_motor_speed(real_t speed);

    ///
    /// Get the pose of the robot
    ///
    const CHRONO_RobotPose& get_pose() const{return pose_; }

    ///
    /// Add the components of this robot to the given chrono::ChSystemNSC
    /// @param sys
    void add_to_sys(chrono::ChSystemNSC& sys) const;

    /// Read/write reference to the senosr manager for this robot
    /// @return
    ///
    RobotSensorManager & get_sensor_manager(){return sensor_manager_;}


private:

    /// the components of the robot
    std::shared_ptr<chrono::ChBodyEasyBox> chassis_;
    std::shared_ptr<chrono::ChBodyEasyCylinder> left_wheel_;
    std::shared_ptr<chrono::ChBodyEasyCylinder> right_wheel_;
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> left_wheel_motor_;
    std::shared_ptr<chrono::ChFunctionConst> left_wheel_motor_speed_func_;
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> right_wheel_motor_;
    std::shared_ptr<chrono::ChFunctionConst> right_wheel_motor_speed_func_;
    std::shared_ptr<chrono::ChBodyEasySphere> caster_wheel_;
    std::shared_ptr<chrono::ChLinkLockRevolute> caster_link_joint_;

    /// The pose of the chassis
    CHRONO_RobotPose pose_;
    RobotData data_;
    RobotSensorManager sensor_manager_;

    bool is_loaded_{false};
};
}
}

#endif

#endif //DIFF_DRIVE_ROBOT_H
