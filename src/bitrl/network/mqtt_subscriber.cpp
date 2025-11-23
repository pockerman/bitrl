//
// Created by alex on 11/23/25.
//


#include "bitrl/network/mqtt_subscriber.h"

#include <optional>
#include <mqtt/message.h>

namespace bitrl {
    namespace network
    {

        MqttSubscriber::MqttSubscriber(const std::string& server, const std::string& topic)
            :
        server_(server),
        topic_(topic),
        cli_(server, "")
        {}

        MqttSubscriber::~MqttSubscriber() {
            try
            {
                cli_.stop_consuming();
                cli_.disconnect()->wait();
            }
            catch (...)
            {

            }
        }

        void
        MqttSubscriber::connect()
        {
            conn_opts_.set_clean_session(true);
            cli_.connect(conn_opts_)->wait();
            cli_.set_callback(*this);
            cli_.start_consuming();
            cli_.subscribe(topic_, 1)->wait();

        }

        void
        MqttSubscriber::message_arrived(mqtt::const_message_ptr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(msg->to_string());
            cv_.notify_one(); // Notify any thread waiting in poll()
        }

        std::optional<std::string>
        MqttSubscriber::poll(std::chrono::milliseconds timeout) {

            std::unique_lock<std::mutex> lock(mutex_);

             if (!cv_.wait_for(lock, timeout, [this]{ return !queue_.empty(); })) {
                 // timeout, no message arrived
                 return std::nullopt;
             }

             auto data = queue_.front();
             queue_.pop();
             return data;

        }

    }
}
