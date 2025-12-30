#include "kernel/discretization/vtk_writer.h"
#include "kernel/discretization/dof_manager.h"
#include "kernel/discretization/element.h"
#include "kernel/discretization/element_mesh_iterator.h"
#include "kernel/discretization/mesh.h"
#include "kernel/discretization/mesh_predicates.h"
#include "kernel/discretization/node.h"
#include "kernel/discretization/node_mesh_iterator.h"
#include "kernel/utilities/file_formats.h"
#include "kernel/utilities/predicates.h"

#include "kernel/base/config.h"

#ifdef USE_TRILINOS
#include "kernel/maths/trilinos_epetra_vector.h"
#endif

#include <chrono>
#include <exception>
#include <vector>

namespace kernel
{
namespace numerics
{

VtkWriter::VtkWriter(const std::string &filename, bool open_file)
    : VtkMeshFileWriter(filename, open_file)
{
}

#ifdef USE_TRILINOS

void VtkWriter::write_solution(const Mesh<1> &mesh, const TrilinosEpetraVector &data,
                               const FVDoFManager<1> &manager)
{
}

void VtkWriter::write_solution(const Mesh<2> &mesh, const TrilinosEpetraVector &data,
                               const FVDoFManager<2> &manager)
{

    this->write_mesh(mesh);

    // get the number of active (this should be) elements
    const uint_t n_elements = mesh.n_elements();

    this->file_ << std::endl;
    this->file_ << "CELL_DATA"
                << " " << n_elements << std::endl;
    this->file_ << "SCALARS " << manager.var_name() << " double " << std::endl;
    this->file_ << "LOOKUP_TABLE default" << std::endl;

    ConstElementMeshIterator<Active, Mesh<2>> filter(mesh);

    auto elem_it = filter.begin();
    auto elem_it_end = filter.end();

    for (; elem_it != elem_it_end; elem_it++)
    {

        const Element<2> *e = *elem_it;
        std::vector<DoF> dofs;
        manager.get_dofs(*e, dofs);
        this->file_ << data[dofs[0].id] << std::endl;
    }
    this->file_ << "\n";
}

#endif

void VtkWriter::write_solution(const Mesh<2> &mesh, const DynVec<real_t> &data,
                               const FVDoFManager<2> &manager)
{

    this->write_mesh(mesh);

    // get the number of active (this should be) elements
    const uint_t n_elements = mesh.n_elements();

    this->file_ << std::endl;
    this->file_ << "CELL_DATA"
                << " " << n_elements << std::endl;
    this->file_ << "SCALARS " << manager.var_name() << " double " << std::endl;
    this->file_ << "LOOKUP_TABLE default" << std::endl;

    ConstElementMeshIterator<Active, Mesh<2>> filter(mesh);

    auto elem_it = filter.begin();
    auto elem_it_end = filter.end();

    for (; elem_it != elem_it_end; elem_it++)
    {

        const Element<2> *e = *elem_it;
        std::vector<DoF> dofs;
        manager.get_dofs(*e, dofs);
        this->file_ << data[dofs[0].id] << std::endl;
    }
    this->file_ << "\n";
}

} // namespace numerics

} // namespace kernel
