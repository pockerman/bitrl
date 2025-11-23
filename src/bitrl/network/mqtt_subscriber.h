//
// Created by alex on 11/23/25.
//

#ifndef MQTT_SUBSCRIBER_H
#define MQTT_SUBSCRIBER_H

#include <mqtt/async_client.h>
#include <string>
#include <optional>
#include <mutex>
#include <queue>

namespace bitrl
{
    namespace network
    {
        class MqttSubscriber: public virtual mqtt::callback
        {
        public:

            MqttSubscriber(const std::string& server, const std::string& topic);
            ~MqttSubscriber() override;

            std::string topic()const noexcept{return topic_;}

            void connect();
            std::optional<std::string> poll(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

            void message_arrived(mqtt::const_message_ptr msg) override;
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
