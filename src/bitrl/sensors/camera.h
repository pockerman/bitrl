//
// Created by alex on 11/23/25.
//

#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

namespace bitrl
{
    namespace sensors
    {
        struct CameraReading {
            cv::Mat image;
            static std::optional<CameraReading> parse(const std::string& base64jpeg);
        };
    }
}


#endif //CAMERA_H
