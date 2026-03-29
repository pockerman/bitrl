#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/utils/io/file_formats.h"

namespace bitrl
{
namespace utils::io
{

JSONFileReader::JSONFileReader(const std::string &filename)
    : FileReaderBase(filename, FileFormats::Type::JSON), data_()
{
}

JSONFileReader::JSONFileReader(const std::filesystem::path &filename)
    :
FileReaderBase(filename.string(), FileFormats::Type::JSON), data_()
{}

void JSONFileReader::open()
{
    this->FileReaderBase::open();
    auto &f = this->get_file_stream();
    data_ = json::parse(f);
}

} // namespace utils::io
} // namespace bitrl
