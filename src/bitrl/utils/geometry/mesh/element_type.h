#ifndef ELEMENT_TYPE_H
#define ELEMENT_TYPE_H

#include "bitrl/bitrl_types.h"

namespace bitrl
{
namespace utils
{
namespace geom
{

template <int spacedim> class Element;

struct ElementType
{

    enum class type
    {
        NODE_ELEM,
        EDGE,
        QUAD,
        TRI,
        HEX,
        TET,
        INVALID_ELEMENT_TYPE
    };

    /**
     * \detailed a useful enumeration of the
     * types of the elements we support
     */
    enum class sub_type
    {
        EDGE2,
        EDGE3,
        EDGE4,
        EDGE5,
        EDGE6,
        EDGE7,
        QUAD4,
        QUAD9,
        QUAD8,
        TRI3,
        TRI6,
        HEX8,
        HEX20,
        TET4
    };

    /**
     * get the number of internal nodes per element type
     */
    static uint_t n_internal_nodes_edge(ElementType::sub_type t);

    /**
     * get the sub-type of the element given the number of nodes
     */
    static ElementType::sub_type element_sub_type(uint_t n_nodes);
};

/**
 * placeholders of the element types we support
 */
struct node_elem_type
{
};
struct edge_type
{
};
struct quad_type
{
};
struct tri_type
{
};
struct hex_type
{
};
struct tet_type
{
};

/**
 *\detailed placeholders of the element subtypes we support
 */
struct edge2_type
{
};
struct edge3_type
{
};
struct edge4_type
{
};
struct quad4_type
{
};
struct quad8_type
{
};
struct quad9_type
{
};
struct hex8_type
{
};

namespace biblsimpp_geom_implementation
{
ElementType::sub_type element_sub_type(uint_t n_nodes, edge_type);
ElementType::sub_type element_sub_type(uint_t n_nodes, tri_type);
ElementType::sub_type element_sub_type(uint_t n_nodes, quad_type);
ElementType::sub_type element_sub_type(uint_t n_nodes, hex_type);
ElementType::sub_type element_sub_type(uint_t n_nodes, tet_type);

} // namespace biblsimpp_geom_implementation

} // namespace geom

} // namespace utils
} // namespace bitrl

#endif // ELEMENT_TYPE_H
