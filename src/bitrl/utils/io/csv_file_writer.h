#ifndef CSV_FILE_WRITER_H
#define CSV_FILE_WRITER_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/utils/io/file_writer_base.h"

#include <vector>
#include <tuple>
#include <initializer_list>

namespace bitrl{
namespace utils{
namespace io{

///
/// \brief The CSVWriter class. Handles writing into CSV file format
///
class CSVWriter: public FileWriterBase
{

public:

    ///
    /// \brief  The default column delimiter
    ///
    static char default_delimiter(){return ',';}

    ///
    /// \brief Constructor
    ///
    CSVWriter(	const std::string& filename, 
				char delim=CSVWriter::default_delimiter()); 

    ///
    /// \brief Write the column names
    ///
    void write_column_names(const std::vector<std::string>& col_names,
                            bool write_header=true);

    ///
    /// \brief Write the column names
    ///
    void write_column_names(const std::vector<std::string_view>& col_names,
                            bool write_header=true);

    ///
    /// \brief Write the column names
    ///
    void write_column_names(const std::initializer_list<std::string_view>& col_names,
                            bool write_header=true);

    ///
    /// \brief Write a row of the file
    ///
    template<typename T>
    void write_row(const std::vector<T>& vals);

    ///
    /// \brief Write the given Vec as a row
    ///
    template<typename T>
    void write_row(const DynVec<T>& vals);

    ///
	/// \brief Write the given vector as a column
	///
    template<typename T>
    void write_column_vector(const std::vector<T>& vals);

    ///
    /// \brief Write the given tuple as a row
    ///
    template<typename...T>
    void write_row(const std::tuple<T...>& row);

    ///
    /// \brief Set the delimiter
    ///
    void set_delimiter(char delim)noexcept{delim_=delim;}

    ///
    /// \brief Returns the column delimiter
    ///
    char get_delimiter()const noexcept{return delim_;}

private:

    ///
    /// \brief The delimiter used to separate the values
    ///
    char delim_;

};

template<typename T>
void
CSVWriter::write_row(const std::vector<T>& vals){

    if(!this->is_open()){
        throw std::logic_error("File "+this->file_name_+" is not open");
    }

    auto& f =  this-> get_file_stream();
    for(uint_t c=0; c<vals.size(); ++c){

        f<<vals[c];

        if(c == vals.size()-1){
            f<<std::endl;
        }
        else{
           f<<",";
        }
    }
}

template<typename T>
void
CSVWriter::write_row(const DynVec<T>& vals){

    if(!this->is_open()){
        throw std::logic_error("File "+this->file_name_+" is not open");
    }

    auto& f =  this-> get_file_stream();
    for(uint_t c=0; c < static_cast<uint_t>(vals.size()); ++c){

        f<<vals[c];

        if(c == static_cast<uint_t>(vals.size())-1){
            f<<std::endl;
        }
        else{
           f<<",";
        }
    }
}

template<typename T>
void
CSVWriter::write_column_vector(const std::vector<T>& vals){

    if(!this->is_open()){
        throw std::logic_error("File "+this->file_name_+" is not open");
    }

    auto& f =  this-> get_file_stream();
    for(uint_t c=0; c<vals.size(); ++c){
        f<<vals[c]<<std::endl;
    }
}

template<typename...T>
void
CSVWriter::write_row(const std::tuple<T...>& row){

    auto& f =  this-> get_file_stream();
    std::apply([&](auto&&...args ){((f<<args<<","), ...);}, row);
    f<<std::endl;
}

}
}
}


#endif // CSV_FILE_WRITER_H
