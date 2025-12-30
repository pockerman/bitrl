#include "bitrl/dynamics/dynamics_matrix_descriptor.h"

#include <stdexcept>
namespace bitrl
{
namespace dynamics
{

DynamicsMatrixDescriptor::DynamicsMatrixDescriptor() : matrices_(), vectors_() {}

DynamicsMatrixDescriptor::matrix_iterator
DynamicsMatrixDescriptor::find_matrix(const std::string &name)
{
    return matrices_.find(name);
}

DynamicsMatrixDescriptor::const_matrix_iterator
DynamicsMatrixDescriptor::find_matrix(const std::string &name) const
{
    return matrices_.find(name);
}

DynamicsMatrixDescriptor::vector_iterator
DynamicsMatrixDescriptor::find_vector(const std::string &name)
{
    return vectors_.find(name);
}

DynamicsMatrixDescriptor::const_vector_iterator
DynamicsMatrixDescriptor::find_vector(const std::string &name) const
{
    return vectors_.find(name);
}

bool DynamicsMatrixDescriptor::has_matrix(const std::string &name) const
{

    auto itr = find_matrix(name);
    return itr != matrices_.end();
}

DynamicsMatrixDescriptor::matrix_type &DynamicsMatrixDescriptor::get_matrix(const std::string &name)
{

    auto itr = matrices_.find(name);

    if (itr != matrices_.end())
    {
        return itr->second;
    }

    throw std::logic_error("Matrix " + name + " not found");
}
const DynamicsMatrixDescriptor::matrix_type &
DynamicsMatrixDescriptor::get_matrix(const std::string &name) const
{

    auto itr = matrices_.find(name);

    if (itr != matrices_.end())
    {
        return itr->second;
    }

    throw std::logic_error("Matrix " + name + " not found");
}

DynamicsMatrixDescriptor::vector_type &DynamicsMatrixDescriptor::get_vector(const std::string &name)
{

    auto itr = vectors_.find(name);

    if (itr != vectors_.end())
    {
        return itr->second;
    }

    throw std::logic_error("Vector " + name + " not found");
}
const DynamicsMatrixDescriptor::vector_type &
DynamicsMatrixDescriptor::get_vector(const std::string &name) const
{
    auto itr = vectors_.find(name);

    if (itr != vectors_.end())
    {
        return itr->second;
    }

    throw std::logic_error("Vector " + name + " not found");
}

} // namespace dynamics
} // namespace bitrl
