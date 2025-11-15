// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef FILE_READER_BASE_H
#define FILE_READER_BASE_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/utils//io/file_formats.h"
#include "bitrl/utils//io/file_handler_base.h"
#include "boost/noncopyable.hpp"

#include <string>
#include <fstream>

namespace bitrl{
namespace utils{
namespace io{
/**
 * @todo write docs
 */
class FileReaderBase: public FileHandlerBase<std::ifstream>
{
    public:

    virtual ~FileReaderBase() = default;

    ///
    /// \brief Attempts to open the file for reading
    ///
    virtual void open() override;

    ///
    /// \brief Returns true if the underlying stream
    /// handler has reached the EOF
    ///
    bool eof()const{return this->get_file_stream().eof();}

protected:

    ///
    /// \brief Constructor
    ///
    FileReaderBase(const std::string& file_name, FileFormats::Type t);
				
};

}
}
}
#endif // FILE_READER_BASE_H
