#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/chrono_diff_drive_robot.h"
#include "bitrl/utils/io/json_file_reader.h"
#include <chrono/physics/ChBodyEasy.h>

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

namespace bitrl
{
namespace rb::bitrl_chrono
{
namespace
{
auto build_chassis()
{
    auto chassis = chrono_types::make_shared<chrono::ChBody>();
    chassis->SetMass(10.0);
    chassis->SetInertiaXX(chrono::ChVector3d(0.1, 0.1, 0.1));
    chassis->SetPos(chrono::ChVector3d(0, 0, 0.15));
    chassis->SetFixed(false);
    return chassis;
}
}

void
CHRONO_DiffDriveRobotBase::load_from_json(const std::string& filename)
{
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loading robot from file: " << filename;
#endif
    utils::io::JSONFileReader json_reader(filename);
    json_reader.open();

    // set the gravity acceleration
    sys_.SetGravitationalAcceleration(chrono::ChVector3d(0, 0.0, -9.81));

    auto chassis = build_chassis();
    sys_.Add(chassis);


    name_ = json_reader.template get_value<std::string>("name");

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loaded robot: " << name_;
#endif
}
}
}
#endif

