cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

SET(EXECUTABLE  mqtt_example_1)
SET(SOURCE ${EXECUTABLE}.cpp)

ADD_EXECUTABLE(${EXECUTABLE} ${SOURCE})

TARGET_LINK_LIBRARIES(${EXECUTABLE} PRIVATE bitrllib
                                            ${BITRL_CHRONO_TARGETS}
                                            paho-mqttpp3
                                            paho-mqtt3as
                                            ${OpenCV_LIBS} )

IF(BITRL_WEBOTS)
    TARGET_LINK_LIBRARIES(${EXECUTABLE} CppController)
ENDIF()