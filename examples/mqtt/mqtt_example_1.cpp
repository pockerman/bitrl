//
// Created by alex on 11/22/25.
//

#include "bitrl/bitrl_types.h"
#include "bitrl/network/mqtt_subscriber.h"
//#include "bitrl/sensors/ekf_sensor_fusion.h"
#include "bitrl/sensors/sensor_type_enum.h"
#include "bitrl/sensors/vector_message.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <stdexcept>

#include "../../src/bitrl/sensors/ekf_sensor_fusion.h"


namespace example
{
    using namespace bitrl;

    class ObservationModel
    {

    public:

        // the ExtendedKalmanFilter expects an exposed
        // input_type
        typedef  DynVec<real_t> input_type;

        ObservationModel();

        // simply return the state
        const DynVec<real_t> evaluate(real_t dt, const std::string& sensor_message,
                                      const std::string& topic, const input_type& u)const;

        // get the H or M matrix
        const DynMat<real_t>& get_matrix(const std::string& name)const;

    private:

        DynMat<real_t> H;
        DynMat<real_t> M;
    };

    const DynVec<real_t>
    ObservationModel::evaluate(real_t dt, const std::string& sensor_message,
                                    const std::string& topic, const input_type& u)const
    {
        auto message = bitrl::sensors::EigenVectorMessage<real_t>::parse(sensor_message);

        if (message.has_value())
        {
            return message.value().message;
        }



    }

    const DynMat<real_t>&
    ObservationModel::get_matrix(const std::string& name)const{
        if(name == "H"){
            return H;
        }
        else if(name == "M"){
            return M;
        }

        throw std::logic_error("Invalid matrix name. Name "+name+ " not found");
    }

}

int main() {

    using namespace  example;

    network::MqttSubscriber subscriber("tcp://localhost:1883", "GPS_TOPIC");
    subscriber.connect();

    while(true)
    {
        auto message = subscriber.read();

        if (message.has_value())
        {
            auto message_value = message.value();
            auto vector_message = sensors::EigenVectorMessage<real_t>::parse(message_value);

            if (vector_message.has_value())
            {
                std::cout << "Received: "<<vector_message.value().message << std::endl;
            }
            else
            {
                std::cout << "Received: "<<message_value<< "but couldn't create vector_message" << std::endl;
            }
        }
        else
        {
            std::cout << "No message received." << std::endl;
        }
    }
    //sensors::EKFSensorFusion sensor_fusion;
    //sensor_fusion.add_sensor_topic("tcp://localhost:1883", "camera", sensors::SensorTypeEnum::CAMERA);

    // while (true)
    // {
    //     auto message = sensor_fusion.read_from_topic("camera",
    //         std::chrono::milliseconds(3000));
    //
    //     if (message.has_value())
    //     {
    //         auto reading = sensors::CameraReading::parse(message.value());
    //
    //         if (reading.has_value())
    //         {
    //             auto img = reading.value().image;
    //             // plot the image
    //             if(img.empty()){
    //                 std::cout << "Could not plot the received image: "<< std::endl;
    //                 return 1;
    //             }
    //
    //             cv::imshow("Image", img);
    //
    //             // Wait for a keystroke in the window
    //             cv::waitKey(0);
    //         }
    //
    //     }
    //
    //     std::this_thread::sleep_for(std::chrono::microseconds(200));
    // }

    return 0;
}


