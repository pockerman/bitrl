#include "kernel/discretization/csv_mesh_writer.h"
#include "kernel/discretization/line_mesh.h"
#include "kernel/discretization/mesh_predicates.h"
#include "kernel/discretization/node.h"
#include "kernel/discretization/node_mesh_iterator.h"

namespace kernel
{

CSVMeshWriter::CSVMeshWriter(const std::string &filename, char delim, bool open_file,
                             const std::ios_base::openmode mode)
    : writer_(filename, delim, open_file, mode)
{
}

void CSVMeshWriter::write_mesh(const numerics::LineMesh<2> &mesh)
{

    /// if the file is not open throw
    if (!is_open())
    {
        throw std::logic_error("File " + writer_.get_filename() + " is not open");
    }

    numerics::ConstNodeMeshIterator<numerics::Active, numerics::LineMesh<2>> filter(mesh);
    auto &file_stream = writer_.get_file_stream();

    auto begin = filter.begin();
    auto end = filter.end();

    for (; begin != end; ++begin)
    {
        auto *node = *begin;

        file_stream << (*node)[0] << "," << (*node)[1] << std::endl;
    }

    file_stream.flush();
    file_stream.close();
}

} // namespace kernel
