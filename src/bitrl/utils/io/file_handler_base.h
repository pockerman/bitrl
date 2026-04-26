#ifndef FILE_HANDLER_BASE_H
#define FILE_HANDLER_BASE_H

#include "bitrl/utils/io/file_formats.h"
#include "boost/noncopyable.hpp"

#include <string>
#include <filesystem>

namespace bitrl
{
namespace utils::io
{
/**
 * @class FileHandlerBase
 * @brief Abstract base class for file handling operations.
 *
 * This class provides a common interface and shared functionality for
 * managing file input/output operations. It encapsulates a low-level
 * file handler (e.g., std::ifstream, std::ofstream, or similar types)
 * and enforces a consistent API for derived classes that implement
 * specific file reading or writing behavior.
 *
 * The class is non-copyable (via boost::noncopyable) to prevent
 * unintended duplication of file handles.
 *
 * @tparam HandlerType The underlying file stream/handler type.
 *
 * ### Responsibilities
 * - Store and expose the file name and file format.
 * - Manage the lifecycle of the file stream (open/close).
 * - Provide access to the underlying file handler.
 * - Ensure proper cleanup by closing the file in the destructor.
 *
 * ### Usage
 * This class is intended to be inherited by concrete file handler
 * implementations (e.g., readers or writers). Derived classes must
 * implement the open() method to define how the file is opened.
 *
 * ### Example
 * @code
 * class MyFileReader : public FileHandlerBase<std::ifstream> {
 * public:
 *     using FileHandlerBase::FileHandlerBase;
 *
 *     void open() override {
 *         f_.open(file_name_, std::ios::in);
 *     }
 * };
 * @endcode
 *
 * ### Notes
 * - The destructor automatically calls close() to ensure the file
 *   is properly released.
 * - Calling close() on an already closed file is safe.
 * - The class does not handle error reporting; this should be
 *   implemented in derived classes if needed.
 */
template <typename HandlerType> class FileHandlerBase : private boost::noncopyable
{
  public:
    typedef HandlerType handler_type;

    ///
    /// @brief Destructor. Ensures that the file is properly closed.
    ///
    /// @post The underlying file stream is closed.
    ///
    virtual ~FileHandlerBase();

    ///
    /// @brief Returns the type/format of the file.
    ///
    /// @return The file format as a FileFormats::Type value.
    /// @post Does not modify object state.
    ///
    FileFormats::Type get_type() const noexcept { return t_; }

    ///
    /// @brief Provides mutable access to the underlying file stream.
    ///
    /// @return Reference to the underlying file handler.
    /// @pre The file may or may not be open.
    ///
    handler_type &get_file_stream() noexcept { return f_; }

    ///
    /// @brief Provides read-only access to the underlying file stream.
    ///
    /// @return Const reference to the underlying file handler.
    ///
    const handler_type &get_file_stream() const noexcept { return f_; }

    ///
    /// @brief Returns the file name associated with this handler.
    ///
    /// @return The file name as a std::string.
    ///
    std::string get_filename() const noexcept { return file_name_; }

    ///
    /// @brief Returns the file path associated with this handler.
    ///
    /// @return The file name as a std::filesystem::path object.
    ///
    std::filesystem::path get_file_path() const noexcept { return file_name_; }

    ///
    /// @brief Checks whether the file is currently open.
    ///
    /// @return true if the file stream is open, false otherwise.
    ///
    bool is_open() const noexcept { return f_.is_open(); }

    ///
    /// @brief Closes the file if it is currently open.
    ///
    /// If the underlying file stream is open, this function attempts
    /// to close it. If the file is already closed, no action is taken.
    ///
    /// @post The file stream is not open (is_open() == false).
    /// \throws May propagate exceptions thrown by handler_type::close()
    ///         if the underlying stream supports throwing exceptions.
    /// \note Calling this function multiple times is safe.
    ///
    virtual void close();

    ///
    /// @brief Opens the file.
    ///
    /// This is a pure virtual function that must be implemented by
    /// derived classes. Implementations should define how the file
    /// is opened (e.g., read/write mode, binary/text mode, etc.).
    ///
    /// @pre The file is not already open, unless the derived class
    ///      explicitly supports reopening.
    /// @post If successful, is_open() == true.
    /// @throws Implementation-defined. Derived classes should document
    ///         any exceptions or error-handling behavior.
    ///
    virtual void open() = 0;

  protected:
    /**
     * @brief Protected constructor.
     *
     * Initializes the file handler with a file name and format.
     *
     * @param file_name The name (or path) of the file to be handled.
     * @param t The file format/type.
     *
     * @pre file_name should not be empty.
     * @post The handler is initialized but the file is not open.
     */
    FileHandlerBase(const std::string &file_name, FileFormats::Type t);

    /**
    * @brief The name of the file to read from or write to.
    *
    * Represents the path used when opening the file.
    */
    std::string file_name_;

    ///
    /// \brief The format/type of the file.
    ///
    /// Immutable after construction.
    ///
    const FileFormats::Type t_;

    /**
     * @brief The underlying file stream/handler.
     *
     * This object represents the low-level file interface
     * (e.g., std::ifstream, std::ofstream, etc.).
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

} // namespace utils::io
} // namespace bitrl

#endif // FILE_HANDLER_BASE_H
