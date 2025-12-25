// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef FILE_HANDLER_BASE_H
#define FILE_HANDLER_BASE_H

#include "bitrl/utils/io/file_formats.h"
#include "boost/noncopyable.hpp"

#include <string>

namespace bitrl
{
namespace utils
{
namespace io
{
/**
 * @todo write docs
 */
template <typename HandlerType> class FileHandlerBase : private boost::noncopyable
{
  public:
    typedef HandlerType handler_type;

    ///
    /// \brief Constructor
    ///
    virtual ~FileHandlerBase();

    ///
    /// \brief Returns the type of the file
    ///
    FileFormats::Type get_type() const noexcept { return t_; }

    ///
    /// \brief Returns the underlying file stream
    ///
    handler_type &get_file_stream() noexcept { return f_; }

    ///
    /// \brief Returns the underlying file stream
    ///
    const handler_type &get_file_stream() const noexcept { return f_; }

    ///
    /// \brief Returns the filename that is used to write
    ///
    std::string get_filename() const noexcept { return file_name_; }

    /**
     * \brief Return true if and only if the file is open
     *
     */
    bool is_open() const noexcept { return f_.is_open(); }

    /**
     * \brief Close the file. Return true if and only if the file
     * was closed successfully false otherwise
     *
     */
    virtual void close();

    /**
     * @brief Open the file
     */
    virtual void open() = 0;

  protected:
    /**
     * @brief protected Constructor so that explicit instantiation of
     * the class fails
     */
    FileHandlerBase(const std::string &file_name, FileFormats::Type t);

    /**
     * @brief The name of the file to write
     *
     */

    std::string file_name_;

    ///
    /// \brief The format of the file
    ///
    const FileFormats::Type t_;

    /**
     * @brief The low level file handler
     */
    handler_type f_;
};

template <typename HandlerType>
FileHandlerBase<HandlerType>::FileHandlerBase(const std::string &file_name, FileFormats::Type t)
    : file_name_(file_name), t_(t)
{
}

template <typename HandlerType> FileHandlerBase<HandlerType>::~FileHandlerBase() { close(); }

template <typename HandlerType> void FileHandlerBase<HandlerType>::close()
{

    if (f_.is_open())
        f_.close();
}

} // namespace io
} // namespace utils
} // namespace bitrl

#endif // FILE_HANDLER_BASE_H
