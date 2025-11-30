//
// Created by alex on 11/23/25.
//

#ifndef EKF_SENSOR_FUSION_H
#define EKF_SENSOR_FUSION_H

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/sensor_type_enum.h"

#include <string>
#include <memory>
#include <unordered_map>


namespace bitrl
{
    namespace network
    {
        // forward declaration
        class MqttSubscriber;
    }
    namespace sensors
    {

        class EKFSensorFusion
        {
        public:

            typedef std::string topic_type;

            ///
            ///
            EKFSensorFusion();
            ~EKFSensorFusion();

            void step(real_t dt);

            std::optional<std::string> read_from_topic(const std::string& topic,
                std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

            void add_sensor_topic(const std::string& mqtt_url,
                                  const topic_type& topic,
                                  SensorTypeEnum sensor_type);

        private:

            std::unordered_map<topic_type, std::unique_ptr<bitrl::network::MqttSubscriber>> sensors_;
            std::unordered_map<topic_type, SensorTypeEnum> topic_to_sensor_type_;

        };
    }
}

#endif //SENSOR_FUSION_H
