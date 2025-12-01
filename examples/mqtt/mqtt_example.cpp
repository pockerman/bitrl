//
// Created by alex on 11/22/25.
//


#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/camera.h"
#include "bitrl/sensors/ekf_sensor_fusion.h"
#include "bitrl/sensors/sensor_type_enum.h"

#include <iostream>
#include <thread>
#include <chrono>

int main() {

    using namespace bitrl;

    sensors::EKFSensorFusion sensor_fusion;
    sensor_fusion.add_sensor_topic("tcp://localhost:1883", "camera", sensors::SensorTypeEnum::CAMERA);

    while (true)
    {
        auto message = sensor_fusion.read_from_topic("camera",
            std::chrono::milliseconds(3000));

        if (message.has_value())
        {
            auto reading = sensors::CameraReading::parse(message.value());

            if (reading.has_value())
            {
                auto img = reading.value().image;
                // plot the image
                if(img.empty()){
                    std::cout << "Could not plot the received image: "<< std::endl;
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

