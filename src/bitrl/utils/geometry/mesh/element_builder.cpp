#include "bitrl/utils/geometry/mesh/element_builder.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/geometry/mesh/edge_element.h"
#include "bitrl/utils/geometry/mesh/quad_element.h"

#include <exception>

namespace bitrl
{
namespace utils
{
namespace geom
{

template <> Element<1> *ElementBuilder<1>::build_element(ElementType::sub_type t)
{

    Element<1> *ptr = nullptr;
    ptr = new EdgeElem<1>(bitrl::consts::INVALID_ID);
    return ptr;
}

template <> Element<2> *ElementBuilder<2>::build_element(ElementType::sub_type t)
{

    Element<2> *ptr = nullptr;
    ptr = new Quad<2>();
    return ptr;
}

template <> EdgeBuilder<1>::edge_ptr_t EdgeBuilder<1>::build_edge(ElementType::sub_type t)
{

    throw std::logic_error("Cannot build edges in 1D");
    EdgeBuilder<1>::edge_ptr_t ptr = nullptr;
    return ptr;
}

template <> EdgeBuilder<2>::edge_ptr_t EdgeBuilder<2>::build_edge(ElementType::sub_type t)
{

    EdgeBuilder<2>::edge_ptr_t ptr = nullptr;
    ptr = new EdgeElem<2>(2);
    return ptr;
}

template <> FaceBuilder<2>::face_ptr_t FaceBuilder<2>::build_face(ElementType::sub_type t)
{
    return EdgeBuilder<2>::build_edge(t);
}

template <> FaceBuilder<1>::face_ptr_t FaceBuilder<1>::build_face(ElementType::sub_type t)
{
    throw std::logic_error("Cannot build faces in 1D");
    FaceBuilder<1>::face_ptr_t ptr = nullptr;
    return ptr;
}

template class ElementBuilder<1>;
template class ElementBuilder<2>;
template class EdgeBuilder<1>;
template class EdgeBuilder<2>;
template class FaceBuilder<1>;
template class FaceBuilder<2>;

} // namespace geom
} // namespace utils
} // namespace bitrl