//
// Created by alex on 8/2/25.
//

#ifndef ARDUINO_CONNECTOR_WIFI_BASE_H
#define ARDUINO_CONNECTOR_WIFI_BASE_H

#include "bitrl/boards/arduino/arduino_connector_base.h"


namespace bitrl
{
    namespace boards::arduino
    {
        class ArduinoConnectorWIFIBase: public ArduinoConnectorBase
        {
        public:

            ///
            /// \brief The base type
            ///
            typedef ArduinoConnectorBase base_type;

            /// Constructor
            /// @param port
            /// @param cidx
            /// @param name
            ArduinoConnectorWIFIBase(const std::string& arduino_url);

            ///
            /// @return string representing the port name
            std::string arduino_url()const noexcept{return arduino_url_;}

            ///
            /// \brief Connect to the Arduino board
            ///
            void connect() override;

            ///
            /// \brief Close the connection
            ///
            virtual void close_connection()override;

            ///
            /// \brief Send the command
            ///
            std::string send_cmd(const ArduinoCMDBase& cmd) override;

        private:
            std::string arduino_url_;

        };

    }
}

#endif //ARDUINO_CONNECTOR_WIFI_BASE_H
