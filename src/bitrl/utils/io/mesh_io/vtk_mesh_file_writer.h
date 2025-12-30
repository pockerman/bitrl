#ifndef VTK_MESH_FILE_WRITER_H
#define VTK_MESH_FILE_WRITER_H

#include "kernel/base/types.h"
#include "kernel/utilities/file_writer_base.h"

#include <string>
#include <vector>

namespace kernel
{
namespace numerics
{

struct VtkMeshMeshCellOptions
{
    std::vector<std::string> options;
};

template <int dim> class Mesh;

///
/// \brief Write the given Mesh object in a VTK format
///
class VtkMeshFileWriter : public FileWriterBase
{

  public:
    /// \brief Constructor
    VtkMeshFileWriter(const std::string &filename, bool open_file = false);

    /// \brief Write the mesh into the file specified in
    /// constructor of this class
    void write_mesh(const Mesh<2> &mesh);

    /// \brief Write the mesh into the file specified in
    /// constructor of this class
    void write_mesh(const Mesh<2> &mesh, const VtkMeshMeshCellOptions &options);

    /// \brief Write the mesh into the file specified in the
    /// constructor of this class and also append the
    /// given cell values
    void write_mesh(const Mesh<2> &mesh, const std::vector<real_t> &cell_values,
                    const std::string &option_name);

    /// \brief Write the header of the file. By default some information
    /// such as date and time the file was created is written
    virtual void write_header() override;

  protected:
    virtual void write_option(const Mesh<2> &mesh, const std::string &name);
};

} // namespace numerics

} // namespace kernel

#endif // VTK_MESH_FILE_WRITER_H
