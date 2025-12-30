//
// Created by alex on 11/23/25.
//

#include "camera.h"
#include "bitrl/sensors/camera.h"
#include <boost/beast/core/detail/base64.hpp>
#include <vector>
namespace bitrl
{
namespace sensors
{
std::optional<CameraReading> CameraReading::parse(const std::string &base64jpeg)
{
    // Decode base64
    auto tmp = std::string(base64jpeg);

    // 1) Allocate output buffer big enough
    std::string decoded;
    decoded.resize(tmp.size() * 3 / 4 + 4); // safe upper bound

    auto result = boost::beast::detail::base64::decode(decoded.data(), tmp.data(), tmp.size());

    // trim to actual decoded size
    decoded.resize(result.first);

    std::vector<uchar> buf(decoded.data(), decoded.data() + decoded.size());
    auto image = cv::imdecode(buf, cv::IMREAD_COLOR);

    if (image.empty())
    {
        return std::nullopt;
    }

    CameraReading reading;
    reading.image = image;
    return reading;
}

} // namespace sensors
} // namespace bitrl
