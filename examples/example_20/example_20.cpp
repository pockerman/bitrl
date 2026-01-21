#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT

#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/messages/ultrasound.h"
#include "bitrl/sensors/ultrasonic_sensor.h"
#include "bitrl/sensors/backends/mqtt_ultrasound_backend.h"
#include "bitrl/utils/io/io_utils.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <iomanip>
#include <memory>

namespace example
{


void run_mqtt()
{
    using namespace bitrl;

    network::MqttSubscriber ultrasound_subscriber("tcp://localhost:1883", "ultrasound");
    ultrasound_subscriber.connect();
    auto sensor_backend = std::make_shared<sensors::backends::MQTT_UltrasonicBackend>(ultrasound_subscriber);
    sensor_backend -> set_max_distance(5.0);
    sensor_backend -> set_sensor_units("m");

    sensors::UltrasonicSensor sensor(sensor_backend, "MyUltrasonicSensor");
    sensor.init();

    while (true)
    {
        auto values = sensor.read_values();
        std::cout << "Measured distance: " << values[0] << " in: " <<sensor.sensor_units()<< std::endl;

        // get the last message read
        const auto& read_message = sensor_backend -> last_value_read();

        std::cout<<"Distance received: "<<read_message.distance<<std::endl;
        std::cout<<"Units    received: "<<read_message.unit_str<<std::endl;
        std::cout<<"Generated      at: "<< read_message.source_timestamp<<std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}

}

int main()
{
    example::run_mqtt();
    return 0;
}
#endif


