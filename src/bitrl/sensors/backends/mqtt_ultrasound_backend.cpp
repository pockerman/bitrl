#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT
#include "bitrl/sensors/backends/mqtt_ultrasound_backend.h"
#include "bitrl/utils/io/json_file_reader.h"

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

namespace bitrl
{
namespace sensors::backends
{
const  std::string MQTT_UltrasonicBackend::BACKEND_TYPE="MQTT_UltrasonicBackend";


MQTT_UltrasonicBackend::MQTT_UltrasonicBackend(network::MqttSubscriber& subscriber)
    :
RangeSensorBackendBase(MQTT_UltrasonicBackend::BACKEND_TYPE),
subscriber_ptr_(&subscriber)
{}


void MQTT_UltrasonicBackend::load_from_json(const std::string& filename)
{

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loading sensor backend from file: " << filename;
#endif

    utils::io::JSONFileReader json_reader(filename);
    json_reader.open();

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loaded sensor backend: MQTT_UltrasonicBackend";
#endif
}

std::vector<real_t> MQTT_UltrasonicBackend::read_values()
{
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Reading backend values: "<< this -> backend_type_str();
#endif

    auto message = subscriber_ptr_ -> poll(std::chrono::milliseconds(3000));

    if (message.has_value())
    {
        auto reading = sensors::UltrasoundMessage::parse(message.value());
        if (reading.has_value())
        {
            auto read_message = reading.value();
            last_message_ = read_message;
            return {read_message.distance};
        }
        {
#ifdef BITRL_LOG
            BOOST_LOG_TRIVIAL(warning)<<"Couldn't read sensor values. Return max_distance";
#endif
            return {this -> max_distance()};
        }
    }

#ifdef BITRL_LOG
        BOOST_LOG_TRIVIAL(warning)<<"Couldn't read sensor values. Return max_distance";
#endif
        return {this -> max_distance()};

}

}
}


#endif

