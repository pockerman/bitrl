#ifndef CSV_FILE_READER_H
#define CSV_FILE_READER_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/utils/io/file_reader_base.h"

#include <boost/noncopyable.hpp>
#include <string>
#include <vector>
#include <fstream>

namespace bitrl{
namespace utils{
namespace io{

///
/// \brief The CSVFileReader class
///
class CSVFileReader: public FileReaderBase
{
public:

    ///
    /// \brief The default column delimiter
    ///
    static std::string default_delimeter(){return ",";}

    ///
    /// \brief Constructor
    ///
    CSVFileReader(const std::string& file_path,
                  const std::string delimeter=CSVFileReader::default_delimeter());
				  
    ///
    /// \brief Destructor. Close the file if it is empty
    ///
    ~CSVFileReader();

    ///
    /// \brief Read a line from the file. It attempts to
    /// open the file if the file is not already open
    ///
    std::vector<std::string> read_line();

    ///
    /// \brief read_line_as_uint. Read the line
    /// as uint
    ///
    std::vector<uint_t> read_line_as_uint();


private:

    ///
    /// \brief The delimiter
    ///
    const std::string delimeter_;

    ///
    /// \brief The current row index
    ///
    uint_t current_row_idx_;
};

}
}
}
#endif // CSV_FILE_READER_H
