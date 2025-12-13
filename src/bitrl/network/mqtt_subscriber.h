//
// Created by alex on 11/23/25.
//

#ifndef MQTT_SUBSCRIBER_H
#define MQTT_SUBSCRIBER_H

#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <mqtt/async_client.h>
#include <string>
#include <optional>
#include <mutex>
#include <queue>

namespace bitrl
{
    namespace network
    {
        ///
        /// A thin wrapper over mqtt::async_client
        ///
        class MqttSubscriber: public virtual mqtt::callback
        {
        public:

            MqttSubscriber(const std::string& server, const std::string& topic);
            ~MqttSubscriber() override;

            std::string topic()const noexcept{return topic_;}

            bool is_connected()const noexcept {return cli_.is_connected();}

            void connect();
            std::optional<std::string> poll(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

            std::optional<std::string> read(std::chrono::milliseconds timeout =
                                            std::chrono::milliseconds::zero());

            void message_arrived(mqtt::const_message_ptr msg) override;

            ///  Publish the payload to the topic that this subscriber listens to
            /// @param payload
            /// @param qos
            /// @param retained
            ///
            void publish(const std::string& payload, int_t qos = 1, bool retained = false);

            ///  Publish the JSON payload
            /// @param payload
            /// @param qos
            /// @param retained
            void publish(const nlohmann::json& payload, int_t qos = 1, bool retained = false);
        private:

            const std::string server_;
            const std::string topic_;

            mqtt::async_client cli_;
            mqtt::connect_options conn_opts_;

            std::mutex mutex_;
            std::condition_variable cv_;
            std::queue<std::string> queue_;

        };
    }
}

#endif //MQTT_SUBSCRIBER_H
