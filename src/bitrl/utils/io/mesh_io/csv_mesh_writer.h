#ifndef CSV_MESH_WRITER_H
#define CSV_MESH_WRITER_H

#include "kernel/base/types.h"
#include "kernel/utilities/csv_file_writer.h"
#include "kernel/discretization/node_mesh_iterator.h"
#include "kernel/discretization/mesh_predicates.h"

#include <vector>
#include <tuple>
#include <initializer_list>

namespace kernel{

namespace numerics {
template<int dim> class LineMesh;
}

///
/// \brief The CSVWriter class. Handles writing into CSV file format
///
class CSVMeshWriter
{

public:

    ///
    /// \brief  The default column delimiter
    ///
    static char default_delimiter(){return ',';}

    ///
    /// \brief Constructor
    ///
    CSVMeshWriter(const std::string& filename, char delim=kernel::utilities::CSVWriter::default_delimiter(),
                  bool open_file=false, const std::ios_base::openmode mode=std::ios_base::out);

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
    /// \brief Write the given tuple as a row
    ///
    template<typename...T>
    void write_row(const std::tuple<T...>& row);

    ///
    /// \brief Set the delimiter
    ///
    void set_delimiter(char delim)noexcept{writer_.set_delimiter(delim);}

    ///
    /// \brief Returns the column delimiter
    ///
    char get_delimiter()const noexcept{return writer_.get_delimiter();}
    
    ///
    /// \brief Returns true if the file is open
    ///
    bool is_open()const{return writer_.is_open();}

    ///
    /// \brief Write the given LineMesh into csv format
    ///
    void write_mesh(const numerics::LineMesh<2>& mesh );

    ///
    /// \brief Write the nodes of the given Mesh
    ///
    template<typename MeshTp>
    void write_mesh_nodes(const MeshTp& mesh);

private:

    ///
    /// \brief The object that handles the CSV
    ///
    kernel::utilities::CSVWriter writer_;

};

inline
void
CSVMeshWriter::write_column_names(const std::vector<std::string>& col_names, bool write_header){

   writer_.write_column_names(col_names, write_header); 
}

inline
void
CSVMeshWriter::write_column_names(const std::vector<std::string_view>& col_names, bool write_header){

    writer_.write_column_names(col_names, write_header); 
}

inline
void
CSVMeshWriter::write_column_names(const std::initializer_list<std::string_view>& col_names, bool write_header){

    std::vector<std::string_view> names(col_names);
    write_column_names(names, write_header);
}

template<typename T>
void
CSVMeshWriter::write_row(const std::vector<T>& vals){
	writer_.write_row(vals);
}

template<typename T>
void
CSVMeshWriter::write_row(const DynVec<T>& vals){

    writer_.write_row(vals);
}

template<typename MeshTp>
void
CSVMeshWriter::write_mesh_nodes(const MeshTp& mesh){

    ///if the file is not open throw
    if(!is_open()){
        throw std::logic_error("File " + writer_.get_filename() + " is not open");
    }

    numerics::ConstNodeMeshIterator<numerics::Active, MeshTp> filter(mesh);
    auto& file_stream = writer_.get_file_stream(); 

    auto begin = filter.begin();
    auto end = filter.end();

    for(; begin != end; ++begin){
        auto* node  = *begin;

        for(uint_t d =0; d<MeshTp::dimension; ++d){

           file_stream<<(*node)[d];

            if(d == MeshTp::dimension -1){
                 file_stream<<std::endl;
             }
            else{
                file_stream<<",";
            }
        }
    }

    file_stream.flush();
    file_stream.close();
}


template<typename...T>
void
CSVMeshWriter::write_row(const std::tuple<T...>& row){
	writer_.write_row(row);
}

}


#endif // CSV_FILE_WRITER_H
