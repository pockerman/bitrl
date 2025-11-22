#include "bitrl/utils/io/csv_file_writer.h"

namespace bitrl{
namespace utils{
namespace io{

CSVWriter::CSVWriter(const std::string& filename, char delim)//, const std::ios_base::openmode mode)
                :
FileWriterBase(filename, FileFormats::Type::CSV),
delim_(delim)
{}

void
CSVWriter::write_column_names(const std::vector<std::string>& col_names, bool write_header){

    //if the file is not open
    if(!this->is_open()){
        throw std::logic_error("File "+this->file_name_+" is not open");
    }

    if(write_header){
        this->write_header();
    }

    auto& f =  this-> get_file_stream();
    for(uint_t c=0; c<col_names.size(); ++c){

        if(c == 0){
          f << "#"+col_names[c]<<",";
        }
        else{
           f << col_names[c]<<",";
        }

        if(c == col_names.size()-1){
            f << std::endl;
        }
    }
}

void
CSVWriter::write_column_names(const std::vector<std::string_view>& col_names, bool write_header){

    //if the file is not open
    if(!this->is_open()){
        throw std::logic_error("File "+this->file_name_+" is not open");
    }

    if(write_header){
        this -> write_header();
    }

    auto& f =  this-> get_file_stream();
    for(uint_t c=0; c < col_names.size(); ++c){

        if(c == 0){
          f << "#"+std::string(col_names[c])<<",";
        }
        else{
           f << col_names[c]<<",";
        }

        if(c == col_names.size()-1){
            f << std::endl;
        }
    }
}

void
CSVWriter::write_column_names(const std::initializer_list<std::string_view>& col_names, bool write_header){

    std::vector<std::string_view> names(col_names);
    write_column_names(names, write_header);
}

}
}
}
