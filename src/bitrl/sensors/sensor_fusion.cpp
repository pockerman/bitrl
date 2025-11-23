//
// Created by alex on 11/23/25.
//

#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/sensor_fusion.h"

#include <memory>
#include <iostream>

namespace bitrl
{
    namespace sensors
    {
        SensorFusion::SensorFusion()
            :
        sensors_()
        {}

        SensorFusion::~SensorFusion()
        {}

        void
        SensorFusion::add_sensor_topic(const std::string& mqtt_url,
                                       const std::string& topic,
                                        SensorTypeEnum sensor_type)
        {
            sensors_.insert({topic, std::make_unique<bitrl::network::MqttSubscriber>(mqtt_url, topic)});
            topic_to_sensor_type_.insert({topic, sensor_type});
            sensors_.find(topic)->second->connect();
        }

        std::optional<std::string>
        SensorFusion::read_from_topic(const std::string& topic,
                                     std::chrono::milliseconds timeout )
        {
            std::cout<<"Reading from topic: "<<topic<<std::endl;
            auto topic_itr = sensors_.find(topic);

            if (topic_itr == sensors_.end())
            {
                return std::nullopt;
            }

            std::cout<<"Polling....: "<<topic<<std::endl;
            auto topic_message = topic_itr->second->poll(timeout);

            if (!topic_message.has_value()) {
                std::cout << "No message received on topic: " << topic << std::endl;
                return std::nullopt;
            }

            std::cout<<"Received from topic: "<<topic<<std::endl;
            return topic_message;
        }

        void
        SensorFusion::step(real_t dt)
        {

        }
    }
}
