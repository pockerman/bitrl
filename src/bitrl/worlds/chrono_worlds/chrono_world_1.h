#ifndef CHRONO_WORLD_1_H
#define CHRONO_WORLD_1_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <memory>
#include <filesystem>

namespace bitrl
{
namespace worlds::chrono_worlds
{

class ChronoWorld1
{
public:

    ///
    /// @brief Load the world from the filesystem
    ///
    void load(const std::filesystem::path& filename);

private:

    // box 1
    std::shared_ptr<chrono::ChBodyEasyBox> box1_;
    std::shared_ptr<chrono::ChBodyEasyBox> box2_;
    std::shared_ptr<chrono::ChBodyEasyBox> box3_;
    std::shared_ptr<chrono::ChBodyEasyBox> box4_;
    std::shared_ptr<chrono::ChBodyEasyBox> box5_;
    std::shared_ptr<chrono::ChBodyEasyBox> box6_;

    std::shared_ptr<chrono::ChBodyEasyBox> floor_;

    rb::bitrl_chrono::CHRONO_DiffDriveRobot robot_;

    bool is_loaded_{false};
    friend void assign_to_chrono_system(chrono::ChSystemNSC& system, const ChronoWorld1& world);

};
}
}


#endif





#endif //WORLD_1_H
