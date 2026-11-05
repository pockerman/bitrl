//
// Created by alex on 11/22/25.
//

#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT

#include "bitrl/network/mqtt_subscriber.h"
#include "../../src/bitrl/sensors/messages/camera.h"

#include <chrono>
#include <iostream>
#include <thread>

int main()
{

    using namespace bitrl;

    network::MqttSubscriber camera_subscriber("tcp://localhost:1883", "camera");
    camera_subscriber.connect();

    while (true)
    {
        auto message = camera_subscriber.poll(std::chrono::milliseconds(3000));

        if (message.has_value())
        {
            auto reading = sensors::CameraMessage::parse(message.value());

            if (reading.has_value())
            {
                auto img = reading.value().image;
                // plot the image
                if (img.empty())
                {
                    std::cout << "Could not plot the received image: " << std::endl;
                    return 1;
                }

                cv::imshow("Image", img);

                // Wait for a keystroke in the window
                cv::waitKey(0);
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