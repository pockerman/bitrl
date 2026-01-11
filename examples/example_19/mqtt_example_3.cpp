#include "bitrl/bitrl_config.h"
#ifdef BITRL_MQTT

#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/messages/ultrasound.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <iomanip>

int main()
{

    using namespace bitrl;

    network::MqttSubscriber ultrasound_subscriber("tcp://localhost:1883", "ultrasound");
    ultrasound_subscriber.connect();

    while (true)
    {
        auto message = ultrasound_subscriber.poll(std::chrono::milliseconds(3000));

        if (message.has_value())
        {
            auto reading = sensors::UltrasoundMessage::parse(message.value());
            if (reading.has_value())
            {

                auto read_message = reading.value();

                std::time_t t = std::chrono::system_clock::to_time_t(read_message.source_timestamp);
                std::tm tm = *std::localtime(&t);
                std::cout<<"Distance received: "<<read_message.distance<<std::endl;
                std::cout<<"Units    received: "<<read_message.unit_str<<std::endl;
                std::cout<<"Generated      at: "<< std::put_time(&tm, "%Y-%m-%d %H:%M:%S")<<std::endl;
            }

        }

        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    return 0;
}

#else
#include <iostream>
int main()
{
    std::cerr << "This example requires MQTT and OpenCV to be enable. "
                 "Reconfigure bitrl with ENABLE_MQTT=ON and ENABLE_OPENCV=ON"
              << std::endl;
    return 1;
}
#endif