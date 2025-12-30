#ifndef FILE_WRITER_BASE_H
#define FILE_WRITER_BASE_H

#include "bitrl/utils/io/file_formats.h"
#include "bitrl/utils/io/file_handler_base.h"

#include "boost/noncopyable.hpp"
#include <fstream>
#include <string>

namespace bitrl
{
namespace utils::io
{

///
/// \brief The FileWriterBase class
///
class FileWriterBase : public FileHandlerBase<std::ofstream>
{

  public:
    ///
    /// \brief The default comment mark
    ///
    static char default_comment_mark() { return '#'; }

    ///
    /// \brief Default ctor
    ///
    FileWriterBase() = default;

    ///
    /// \brief Constructor. Construct by passing the name
    /// of the file to write into.
    ///
    FileWriterBase(const std::string &file_name,
                   FileFormats::Type t); // const std::ios_base::openmode mode=std::ios_base::out);

    ///
    /// \brief Destructor
    ///
    virtual ~FileWriterBase();

    ///
    /// \brief Open the file for writing
    ///
    virtual void open() override;

    ///
    /// \brief Return the mark that signifies the beginning of a comment line
    ///
    char get_comment_mark() const { return comment_mark_; }

    ///
    /// \brief Set the comment mark
    ///
    void set_comment_mark(char mark) { comment_mark_ = mark; }

    ///
    /// \brief Write the header of the file. By default some information
    /// such as date and time the file was created is written
    ///
    virtual void write_header();

  protected:
    ///
    /// \brief The mark that signifies the beginning of a comment line. The default is #
    ///
    char comment_mark_;
};

} // namespace utils::io
} // namespace bitrl
#endif // FILE_WRITER_BASE_H
