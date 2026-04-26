#include "bitrl/sensors/backends/range_sensor_backend_base.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"
#include "bitrl/sensors/backends/mqtt_ultrasound_backend.h"
#include "bitrl/utils/std_map_utils.h"

#ifdef BITRL_CHRONO
#include <chrono/assets/ChVisualShapeBox.h>
#endif

#include <stdexcept>
namespace bitrl
{
namespace sensors::backends
{
std::shared_ptr<RangeSensorBackendBase> RangeSensorBackendBase::create(const std::string& backend_type)
{

    if (backend_type == "CHRONO")
    {
        return std::make_shared<CHRONO_UltrasonicBackend>();
    }
    else if (backend_type == "MQTT")
    {
        return std::make_shared<MQTT_UltrasonicBackend>();
    }

    throw std::runtime_error("Unknown backend type: " + backend_type);
}

#ifdef BITRL_CHRONO
void RangeSensorBackendBase::add_visual_shape(const std::unordered_map<std::string, std::any>& shape_properties)
{
    auto x_length = utils::resolve<float_t>("x_length", shape_properties);
    auto y_length = utils::resolve<float_t>("y_length", shape_properties);
    auto z_length = utils::resolve<float_t>("z_length", shape_properties);
    auto color = utils::resolve<chrono::ChColor>("chrono::ChColor", shape_properties);
    auto sensor_frame = utils::resolve<chrono::ChFrame<>>("chrono::ChFrame<>", shape_properties);

    auto sensor_vis = chrono_types::make_shared<chrono::ChVisualShapeBox>(x_length, y_length, z_length); // 4cm cube
    sensor_vis->SetColor(color); //chrono::ChColor(1.0f, 0.0f, 0.0f)); // red

    // chrono::ChFrame<> sensor_frame(
    //     chrono::ChVector3d(cd.chassis_x / 2.0 + 0.02, 0.0, cd.chassis_z / 2.0)
    // );

    // Attach to chassis
    this -> body_ -> AddVisualShape(sensor_vis, sensor_frame);
}
#endif

}
}
