#include "bitrl/sensors/sensor_base.h"

#include <string>

namespace bitrl
{
namespace sensors
{

std::string SensorBase::backend_type_str()const{return bitrl::consts::INVALID_STR;}
std::string SensorBase::sensor_units()const{return bitrl::consts::INVALID_STR;}
}
}