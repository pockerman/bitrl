#ifndef FILE_FORMATS_H
#define FILE_FORMATS_H

#include <string>

namespace bitrl{
namespace utils{
namespace io{	


///
/// Small struct that holds an enumeration for varioufile formats
///

struct FileFormats
{

  ///
  /// File formats types
  ///
  enum class Type{CSV=0, JSON=1, INVALID_TYPE};

  ///
  /// Return an std::string representation of the given file format type
  ///
  static std::string type_to_string(Type t);
};

}
}
}


#endif // FILE_FORMATS_H
