#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"

#include "chrono/collision/ChCollisionSystem.h"

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

namespace bitrl
{
namespace sensors::backends
{

const  std::string ChronoUltrasonicBackend::BACKEND_TYPE = "ChronoUltrasonicBackend";

ChronoUltrasonicBackend::ChronoUltrasonicBackend(chrono::ChSystem& sys_ptr,
                                                std::shared_ptr<chrono::ChBody> body)
    :
RangeSensorBackendBase(ChronoUltrasonicBackend::BACKEND_TYPE),
sys_ptr_(&sys_ptr),
body_(body)
{}

void ChronoUltrasonicBackend::load_from_json(const std::string& filename)
{

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loading sensor backend from file: " << filename;
#endif

    utils::io::JSONFileReader json_reader(filename);
    json_reader.open();

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loaded sensor backend: ChronoUltrasonicBackend";
#endif
}

std::vector<real_t> ChronoUltrasonicBackend::read_values()
{
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Reading backend values: "<< this -> backend_type_str();
#endif

    chrono::ChVector3d start =
             body_->GetFrameRefToAbs().TransformPointLocalToParent(position_);
    chrono::ChVector3d dir =
        body_->GetFrameRefToAbs().GetRotMat().GetAxisX();
    dir.Normalize();

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Reading backend values start: "<< start;
#endif

    chrono::ChVector3d end = start + dir * this -> max_distance();
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Reading backend values end: "<< end;
#endif

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
#ifdef BITRL_LOG
        BOOST_LOG_TRIVIAL(info)<<"A Ray was hit: "<< end;
#endif
        return {(hit.abs_hitPoint - start).Length()};
    }

    return {this -> max_distance()};
}
}
}

#endif
