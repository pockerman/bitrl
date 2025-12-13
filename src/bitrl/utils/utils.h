//
// Created by alex on 12/13/25.
//

#ifndef UTILS_H
#define UTILS_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <string>

namespace bitrl
{
  namespace utils
  {
    /**
     * Generate UUID4 strings using Boost::uuids.
     * For more info on UUIDs see: https://www.ibm.com/docs/en/cobol-zos/6.3.0?topic=functions-uuid4
     * @return
     */
    inline
    std::string uuid4()
    {
      return boost::uuids::to_string(boost::uuids::random_generator()());
    }

  }
}

#endif //UTILS_H
