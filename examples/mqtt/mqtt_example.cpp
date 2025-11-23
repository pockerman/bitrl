//
// Created by alex on 11/22/25.
//

#include <mqtt/async_client.h>
#include <iostream>

int main() {
    mqtt::async_client client("tcp://localhost:1883", "testclient");
    client.connect()->wait();
    client.publish("test/topic", "Hello!", 1, false)->wait();
    std::cout << "Message sent!" << std::endl;
}

