#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"

#include "chrono/collision/ChCollisionSystem.h"



#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

#include <string>

namespace bitrl
{
namespace sensors::backends
{

const  std::string CHRONO_UltrasonicBackend::BACKEND_TYPE = "CHRONO_UltrasonicBackend";

CHRONO_UltrasonicBackend::CHRONO_UltrasonicBackend(chrono::ChSystem& sys_ptr,
                                                std::shared_ptr<chrono::ChBody> body)
    :
RangeSensorBackendBase(CHRONO_UltrasonicBackend::BACKEND_TYPE),
sys_ptr_(&sys_ptr),
body_(body)
{}

void CHRONO_UltrasonicBackend::load_from_json(const std::string& filename)
{

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loading sensor backend from file: " << filename;
#endif

    utils::io::JSONFileReader json_reader(filename);
    json_reader.open();

    auto max_distance = json_reader.at<real_t>("max_distance");
    this -> set_max_distance(max_distance);

    auto min_distance = json_reader.at<real_t>("min_distance");
    this -> set_min_distance(max_distance);

    auto update_rate = json_reader.at<real_t>("update_rate");
    this -> set_sampling_period(update_rate);

    auto name = json_reader.at<std::string>("name");

    auto pos = json_reader.get("mounted_position");
    for (size_t i = 0; i < 3; ++i)
        position_[i] = pos.at(i).get<real_t>();



#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loaded sensor backend: CHRONO_UltrasonicBackend";
#endif
}

std::vector<real_t> CHRONO_UltrasonicBackend::read_values()
{
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Reading backend values: "<< this -> backend_type_str();
#endif

    chrono::ChVector3d start =
             body_->GetFrameRefToAbs().TransformPointLocalToParent(position_);
    chrono::ChVector3d dir =
        body_->GetFrameRefToAbs().GetRotMat().GetAxisX();
    dir.Normalize();

    chrono::ChVector3d end = start + dir * this -> max_distance();

    auto collision_sys = sys_ptr_ -> GetCollisionSystem();
    if (collision_sys == nullptr)
    {
#ifdef BITRL_LOG
        BOOST_LOG_TRIVIAL(warning)<<"Collision system is not enabled for backend. Return max_distance";
#endif
        return {this -> max_distance()};
    }

    chrono::ChCollisionSystem::ChRayhitResult hit;
    if (collision_sys -> RayHit(start, end, hit))
    {
        return {(hit.abs_hitPoint - start).Length()};
    }

    return {this -> max_distance()};
}
}
}

#endif
