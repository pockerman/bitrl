#include "bitrl/boards/arduino/arduino_connector_usb_base.h"

#include <iostream>
#include <string>
#include <chrono>
#include <thread>


/*
    Arduino Code
    String incomingMessage = "";

    void setup() {
      pinMode(LED_BUILTIN, OUTPUT);
      Serial.begin(9600);
    }

    void loop() {
      if (Serial.available()) {
        incomingMessage = Serial.readStringUntil('\n');
        incomingMessage.trim();  // Remove whitespace and newline

        if (incomingMessage == "ON") {
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("LED is ON");
        } else if (incomingMessage == "OFF") {
          digitalWrite(LED_BUILTIN, LOW);
          Serial.println("LED is OFF");
        } else {
          Serial.println("Unknown command");
        }
      }
    }
 */


namespace example
{
    using bitrl::boards::arduino::ArduinoCMDBase;
    using bitrl::boards::arduino::ArduinoConnectorUSBBase;

    struct ArduinoONCMD: public ArduinoCMDBase
    {
        virtual std::string get_cmd()const final;
    };

    std::string ArduinoONCMD::get_cmd() const
    {
        return "ON";
    }

    struct ArduinoOFFCMD: public ArduinoCMDBase
    {
        virtual std::string get_cmd()const final;
    };

    std::string ArduinoOFFCMD::get_cmd() const
    {
        return "OFF";
    }
}


int main() {

    using namespace example;

    ArduinoConnectorUSBBase connector("/dev/ttyACM0");
    connector.connect();

    ArduinoONCMD on_cmd;
    ArduinoOFFCMD off_cmd;

    std::string user_input;
    while (true) {
        std::cout << "Enter command ON/OFF or e (to exit): ";
        std::getline(std::cin, user_input);

        if (user_input == "e") {
            connector.send_cmd(off_cmd);
            break;
        }

        if (user_input == "ON")
        {
            connector.send_cmd(on_cmd);
        }

        if (user_input == "OFF")
        {
            connector.send_cmd(off_cmd);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    connector.close_connection();
    return 0;
}
