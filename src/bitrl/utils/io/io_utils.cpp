#include "bitrl/utils/io/io_utils.h"

#include <iomanip>
#include <chrono>

namespace bitrl
{
namespace utils::io
{
std::ostream& print_time_point(std::ostream &out, const std::chrono::system_clock::time_point& tp)
{
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::tm tm = *std::localtime(&t);
    out<< std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return out;
}
}
}
