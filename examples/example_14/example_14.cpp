#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_diff_drive_robot.h"

namespace example14
{
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobotBase;
}

int main()
{
    using namespace example14;

    CHRONO_DiffDriveRobotBase robot;
    robot.load_from_json(bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json");


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
