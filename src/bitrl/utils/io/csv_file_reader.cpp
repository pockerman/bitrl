#include "bitrl/utils/io/csv_file_reader.h"
#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/utils/io/file_formats.h"

#include <boost/algorithm/string.hpp>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace utils
{
namespace io
{

CSVFileReader::CSVFileReader(const std::string &file_name, const std::string delimeter)
    : FileReaderBase(file_name, FileFormats::Type::CSV), delimeter_(delimeter), current_row_idx_(0)
{
}

CSVFileReader::~CSVFileReader() { close(); }

std::vector<std::string> CSVFileReader::read_line()
{

    if (!this->is_open())
    {
        throw std::logic_error("Attempt to read from a file that is not open");
    }

    auto &f = this->get_file_stream();

#ifdef BITRL_DEBUG

    if (!f.good())
    {
        std::string msg("Failed to open file: ");
        msg += file_name_;
        assert(false && msg.c_str());
    }
#endif

    std::vector<std::string> result;
    if (f.eof())
    {
        return result;
    }

    std::string line = "";
    std::getline(f, line);

    boost::algorithm::split(result, line, boost::is_any_of(delimeter_));
    current_row_idx_++;
    return result;
}

std::vector<uint_t> CSVFileReader::read_line_as_uint()
{

    auto line = read_line();
    std::vector<uint_t> line_int(line.size());

    for (uint_t i = 0; i < line.size(); ++i)
    {
        line_int[i] = std::stoull(line[i]);
    }

    return line_int;
}

} // namespace io
} // namespace utils
} // namespace bitrl
