#ifndef CHRONO_ROBOT_POSE_H
#define CHRONO_ROBOT_POSE_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO
#include "bitrl/bitrl_types.h"
#include <chrono/physics/ChBody.h>
#include <string>
namespace bitrl
{
namespace rb::bitrl_chrono
{

/**
 * @class CHRONO_RobotPose
 * @ingroup rb_chrono
 * @brief Wrapper for representing the state of a CHRONO_Robot
 */
class CHRONO_RobotPose
{
public:

    /**
     *@brief Constructr
     */
    explicit CHRONO_RobotPose(std::shared_ptr<chrono::ChBody> robot_ptr=nullptr);

    /**
     *
     * @param robot_ptr
     */
    void set_body(std::shared_ptr<chrono::ChBody> robot_ptr){robot_ptr_ = robot_ptr;}

    /**
     * @brief Return the position vector
     */
    chrono::ChVector3d position(){return robot_ptr_ -> GetPos();}

    /**
     * @brief Return the rotation matrix
     */
    chrono::ChMatrix33d rotation_matrix() const {return robot_ptr_ -> GetRotMat();}

    /**
     * @brief Return the transformation of the local to the world coordinats
     */
    chrono::ChVector3d to_world_coords(chrono::ChVector3d point){return robot_ptr_ -> GetFrameRefToAbs().TransformPointLocalToParent(point);}

    /**
     * @brief Transform the world point to local coordinates
     */
    chrono::ChVector3d to_local_coords(chrono::ChVector3d point){return robot_ptr_ -> GetFrameRefToAbs().TransformPointParentToLocal(point);}
private:

    /**
     * @brief Pointer to the robot
     */
    std::shared_ptr<chrono::ChBody> robot_ptr_;

};

inline
CHRONO_RobotPose::CHRONO_RobotPose(std::shared_ptr<chrono::ChBody> robot_ptr)
    :
robot_ptr_(robot_ptr)
{}
}
}

#endif

#endif //CHRONO_ROBOT_STATE_H
