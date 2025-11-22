// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef JSON_FILE_READER_H
#define JSON_FILE_READER_H

#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/utils/io/file_reader_base.h"

namespace bitrl{
namespace utils::io
{
    /**
 * @todo write docs
 */
    class JSONFileReader final: public FileReaderBase
    {

    public:


        JSONFileReader(const std::string& filename);

        ///
    /// \brief Attempts to open the file for reading
    ///
        virtual void open() override final;

        /**
     * @brief Get the value specified by the label
     */
        template<typename T>
        T get_value(const std::string& label)const;

    private:

        using json = nlohmann::json;
        json data_;


    };


    template<typename T>
    T
    JSONFileReader::get_value(const std::string& label)const{

        if(!this -> is_open()){
            throw std::logic_error("JSON file is not open. Have you called open()?");
        }
        return data_[label].template get<T>();

    }

}
}
#endif // JSON_FILE_READER_H
