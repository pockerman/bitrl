//
// Created by alex on 7/5/25.
//

#include  "bitrl/boards/arduino/arduino_connector_wifi_base.h"
#include <iostream>

#include <thread>
#include <chrono>


/**
 * // Code for Arduino
 *
#include <WiFiS3.h>

char ssid[] = "YOU-SSID";
char password[] = "YOUR-PASSWORD";

int status = WL_IDLE_STATUS;
WiFiServer server(8005);


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  delay(1000);
  if (Serial.available()) {
    Serial.println("Connecting to WiFi...");
  }

  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, password);
    delay(5000);
  }

  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("HTTP server started");

}

void loop() {

  WiFiClient client = server.available();

  if (client) {
    //client.flush();
    Serial.println("Client connected.");
    String request = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\n' && request.endsWith("\r\n\r\n")) {
          break; // End of HTTP headers
        }
      }
    }

    String body = "";
    while (client.available()) {
      body += (char)client.read();
    }

    if(body == "ON"){
      digitalWrite(LED_BUILTIN, HIGH);
      String response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nLED is ON\n";
      client.print(response);
    }
    else if(body == "OFF"){
      digitalWrite(LED_BUILTIN, LOW);
      String response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nLED is OFF\n";
      client.print(response);

    }

    delay(1);
    client.stop();
    Serial.println("Client disconnected.");
  }
}

 */
namespace example
{
  using bitrl::boards::arduino::ArduinoCMDBase;
  using bitrl::boards::arduino::ArduinoConnectorWIFIBase;

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

  ArduinoConnectorWIFIBase connector("http://192.168.0.70:8005");
  ArduinoONCMD on_cmd;
  ArduinoOFFCMD off_cmd;

  std::string user_input;
  while (true)
  {
    std::cout << "Enter command ON/OFF or e (to exit): ";
    std::getline(std::cin, user_input);

    if (user_input == "e") {
      connector.send_cmd(off_cmd);
      break;
    }

    if (user_input == "ON")
    {
      auto response = connector.send_cmd(on_cmd);
      std::cout<<"Arduino response: "<<response<<std::endl;
    }

    if (user_input == "OFF")
    {
      auto response = connector.send_cmd(off_cmd);
      std::cout<<"Arduino response: "<<response<<std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

    return 0;
}

