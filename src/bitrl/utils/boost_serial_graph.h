#ifndef BOOST_SERIAL_GRAPH_H
#define BOOST_SERIAL_GRAPH_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_graph.hpp>

#include <utility>
#include <vector>
#include <iterator>
#include <stdexcept>

namespace bitrl
{
namespace utils{

///
/// \brief. Representation of a graph using adjacency lists.
/// The underlying implementation uses Boost Graph library.
/// This wrapper is introduced to simplify the creation
/// and handling of the graph.
///
template<typename VertexData, typename EdgeData>
class BoostSerialGraph
{

public:

    ///
    /// \brief vertex_data_t The type of the vertex data
    ///
    typedef VertexData vertex_data_t;

    ///
    /// \brief edge_data_t The type of the edge data
    ///
    typedef EdgeData   edge_data_t;

    ///
    /// \brief Class that represents the Node of a graph
    ///
    struct SerialGraphNode
    {
        vertex_data_t data;
        uint_t  id{bitrl::consts::INVALID_ID};

        SerialGraphNode()=default;
        SerialGraphNode(const vertex_data_t& data);
        SerialGraphNode(vertex_data_t&& data);
        SerialGraphNode(const SerialGraphNode& o);

        SerialGraphNode& operator=(const SerialGraphNode& o);
        bool operator==(const SerialGraphNode& o)const{return this->id==o.id;}
        bool operator!=(const SerialGraphNode& o)const{return !(*this==o);}
    };

    ///
    /// \brief vertex_t The vertex type
    ///
    typedef SerialGraphNode vertex_t;

    ///
    /// \brief edge_t The edge type
    ///
    typedef GenericLine<vertex_t, EdgeData> edge_t;

private:

    typedef boost::undirected_graph<vertex_t, edge_t> graph_type;
    typedef typename boost::graph_traits<graph_type>::vertex_descriptor vertex_descriptor_t;
    typedef typename boost::graph_traits<graph_type>::edge_descriptor edge_descriptor_t;

public:

    ///
    /// \brief edge_iterator Edge iterator
    ///
    typedef typename graph_type::edge_iterator edge_iterator;

    ///
    /// \brief adjacency_iterator Adjacency iterator
    ///
    typedef typename graph_type::adjacency_iterator adjacency_iterator;

    ///
    /// \brief Constructor
    ///
    explicit BoostSerialGraph(uint_t nvs=0);

    ///
    /// \brief Add a vertex to the graph by providing the data
    ///
    vertex_t& add_vertex(const VertexData& data);

    ///
    /// \brief Add an edge formed by the two given vertices
    ///
    edge_t& add_edge(uint_t v1, uint_t v2);

    ///
    /// \brief Access the i-th vertex of the graph
    ///
    const vertex_t& get_vertex(uint_t i)const;

    ///
    /// \brief Access the i-th vertex of the graph
    ///
    vertex_t& get_vertex(uint_t i);

    ///
    /// \brief Access the vertex given the vertex descriptor
    /// This is needed when accessing the vertices using the adjacency_iterator
    ///
    vertex_t& get_vertex(adjacency_iterator itr);

    ///
    /// \brief Access the vertex given the vertex descriptor
    /// This is needed when accessing the vertices using the adjacency_iterator
    ///
    const vertex_t& get_vertex(adjacency_iterator itr)const;

    ///
    /// \brief Access the i-th edge of the graph with endpoints
    /// the given vertices
    ///
    const edge_t& get_edge(uint_t v1, uint_t v2)const;

    ///
    /// \brief Access the i-th edge of the graph with endpoints
    /// the given vertices
    ///
    edge_t& get_edge(uint_t v1, uint_t v2);

    ///
    /// \brief edges Access the edges of the tree
    ///
    std::pair<edge_iterator,edge_iterator> edges()const;

    ///
    /// \brief Returns the neighboring vertices for the given vertex id
    ///
    std::pair<adjacency_iterator, adjacency_iterator> get_vertex_neighbors(uint_t id)const;

    ///
    /// \brief Returns the neighboring vertices for the given vertex id
    ///
    std::pair<adjacency_iterator, adjacency_iterator> get_vertex_neighbors(const vertex_t& v)const;

    ///
    /// \brief get_vertex_neighbors_ids Returns the ids of the vertices
    /// connectected with this vertex
    ///
    std::vector<uint_t> get_vertex_neighbors_ids(uint_t id)const;

    ///
    /// \brief Returns the number of vertices
    ///
    uint_t n_vertices()const{return g_.num_vertices();}

    ///
    /// \brief Returns the maximum vertex index
    ///
    uint_t max_vertex_index()const noexcept{return g_.max_vertex_index();}

    ///
    /// \brief Returns the number of edges
    ///
    uint_t n_edges()const{return g_.num_edges();}

    ///
    /// \brief Returns the maximum edge index
    ///
    uint_t max_edge_index() const noexcept{return g_.max_edge_index();}

    ///
    /// \brief Clear the graph
    ///
    void clear(){g_.clear();}

private:

    ///
    /// \brief The actual graph
    ///
    graph_type g_;
};

template<typename VertexData,typename EdgeData>
BoostSerialGraph<VertexData,EdgeData>::BoostSerialGraph(uint_t nv)
:
g_(nv)
{}

template<typename VertexData, typename EdgeData>
typename BoostSerialGraph<VertexData,EdgeData>::vertex_t&
BoostSerialGraph<VertexData,EdgeData>::add_vertex(const VertexData& data){

    typedef typename BoostSerialGraph<VertexData,EdgeData>::vertex_t vertex_t;
    typedef typename BoostSerialGraph<VertexData,EdgeData>::vertex_descriptor_t    vertex_descriptor_t;
    uint_t idx = n_vertices();

    //add a new vertex
    vertex_descriptor_t a = boost::add_vertex(g_);
    vertex_t& v = g_[a];
    v.data = data;
    v.id = idx;
    return v;
}

template<typename VertexData,typename EdgeData>
typename BoostSerialGraph<VertexData,EdgeData>::edge_t&
BoostSerialGraph<VertexData,EdgeData>::add_edge(uint_t v1, uint_t v2){

    //TODO what happens when v1 and v2 are not valid vertex indices?
    //at the moment we throw an assertion
    if(v1>=n_vertices() || v2 >=n_vertices())
        throw std::logic_error("Invalid vertex index v1/v2: "+
                                std::to_string(v1)+
                                "/"+
                                std::to_string(v2)+
                                " not in [0,"+
                                std::to_string(n_vertices())+
                                ")");
    typedef typename BoostSerialGraph<VertexData,EdgeData>::vertex_descriptor_t vertex_descriptor_t;
    typedef typename BoostSerialGraph<VertexData,EdgeData>::edge_descriptor_t edge_descriptor_t;
    typedef typename BoostSerialGraph<VertexData,EdgeData>::edge_t edge_t;
    edge_descriptor_t et;
    bool condition;

    // get the vertices that correspond to the indices
    vertex_descriptor_t a = boost::vertex(v1, g_);
    vertex_descriptor_t b = boost::vertex(v2, g_);
    uint_t idx = n_edges();

    // create an edge
    boost::tie(et, condition) = boost::add_edge(a,b,g_);
    edge_t& edge = g_[et];
    edge.set_id(idx);
    return edge;
}


template<typename VertexData,typename EdgeData>
const typename BoostSerialGraph<VertexData,EdgeData>::vertex_t&
BoostSerialGraph<VertexData,EdgeData>::get_vertex(uint_t i)const{

    if(i>=n_vertices()){
        throw std::logic_error("Invalid vertex index. Index "+
                                std::to_string(i)+
                                " not in [0,"+
                                std::to_string(n_vertices())+
                                ")");
    }

    typedef typename BoostSerialGraph<VertexData,EdgeData>::vertex_descriptor_t vertex_descriptor_t;
    vertex_descriptor_t a = boost::vertex(i,g_);
    return g_[a];
}

template<typename VertexData,typename EdgeData>
typename BoostSerialGraph<VertexData,EdgeData>::vertex_t&
BoostSerialGraph<VertexData,EdgeData>::get_vertex(adjacency_iterator itr){

    return g_[*itr];
}

template<typename VertexData,typename EdgeData>
const typename BoostSerialGraph<VertexData,EdgeData>::vertex_t&
BoostSerialGraph<VertexData,EdgeData>::get_vertex(adjacency_iterator itr)const{

    return g_[*itr];
}

template<typename VertexData,typename EdgeData>
typename BoostSerialGraph<VertexData,EdgeData>::vertex_t&
BoostSerialGraph<VertexData,EdgeData>::get_vertex(uint_t i){

    return const_cast<BoostSerialGraph<VertexData,EdgeData>::vertex_t&>(

            static_cast<const BoostSerialGraph<VertexData,EdgeData>&>(*this).get_vertex(i)
    );
}

template<typename VertexData,typename EdgeData>
std::vector<uint_t>
BoostSerialGraph<VertexData,EdgeData>::get_vertex_neighbors_ids(uint_t id)const{

    if(id >=n_vertices()){
        throw std::logic_error("Invalid vertex index. Index "+
                                std::to_string(id)+
                                " not in [0,"+
                                std::to_string(n_vertices())+
                                ")");
    }


    auto vneighs = get_vertex_neighbors(id);
    std::vector<uint_t> neighbors;
    neighbors.reserve(std::distance(vneighs.first, vneighs.second));

    auto start = vneighs.first;
    auto end = vneighs.second;
    while(start != end){
        auto& vertex = get_vertex(start);
        neighbors.push_back(vertex.id);
        ++start;
    }

    neighbors;
}


template<typename VertexData,typename EdgeData>
const typename BoostSerialGraph<VertexData,EdgeData>::edge_t&
BoostSerialGraph<VertexData,EdgeData>::get_edge(uint_t v1, uint_t v2)const{

    //TODO what happens when v1 and v2 are not valid vertex indices?
    //at the moment we throw an assertion
    if(v1>=n_vertices() || v2 >=n_vertices())
        throw std::logic_error("Invalid vertex index v1/v2. Indeces "+
                                std::to_string(v1)+
                                "/"+
                                std::to_string(v2)+
                                " not in [0,"+
                                std::to_string(n_vertices())+
                                ")");

    typedef typename BoostSerialGraph<VertexData,EdgeData>::vertex_descriptor_t vertex_descriptor_t;
    typedef typename BoostSerialGraph<VertexData,EdgeData>::edge_descriptor_t edge_descriptor_t;
    vertex_descriptor_t a = boost::vertex(v1,g_);
    vertex_descriptor_t b = boost::vertex(v2,g_);

    std::pair<edge_descriptor_t,bool> rslt = boost::edge(a,b,g_);

    return g_[rslt.first];
}


template<typename VertexData,typename EdgeData>
typename BoostSerialGraph<VertexData,EdgeData>::edge_t&
BoostSerialGraph<VertexData,EdgeData>::get_edge(uint_t v1, uint_t v2){

    return const_cast<BoostSerialGraph<VertexData,EdgeData>::edge_t&>(
                static_cast<const BoostSerialGraph<VertexData,EdgeData>&>(*this).get_edge(v1,v2)
        );

}

template<typename VertexData,typename EdgeData>
std::pair<typename BoostSerialGraph<VertexData,EdgeData>::adjacency_iterator,
          typename BoostSerialGraph<VertexData,EdgeData>::adjacency_iterator>
BoostSerialGraph<VertexData,EdgeData>::get_vertex_neighbors(uint_t i)const{

    if(i>=n_vertices()){
        throw std::logic_error("Invalid vertex index. Index "+
                                std::to_string(i)+
                                " not in [0,"+
                                std::to_string(n_vertices())+
                                ")");
    }

    typedef typename BoostSerialGraph<VertexData,EdgeData>::vertex_descriptor_t vertex_descriptor_t;
    vertex_descriptor_t a = boost::vertex(i,g_);
    return boost::adjacent_vertices(a, g_);
}

template<typename VertexData,typename EdgeData>
std::pair<typename BoostSerialGraph<VertexData,EdgeData>::adjacency_iterator,
          typename BoostSerialGraph<VertexData,EdgeData>::adjacency_iterator>
BoostSerialGraph<VertexData,EdgeData>::get_vertex_neighbors(const typename BoostSerialGraph<VertexData,EdgeData>::vertex_t& v)const{
    return get_vertex_neighbors(v.id);
}

template<typename VertexData,typename EdgeData>
std::pair<typename BoostSerialGraph<VertexData,EdgeData>::edge_iterator,
          typename BoostSerialGraph<VertexData,EdgeData>::edge_iterator>
BoostSerialGraph<VertexData,EdgeData>::edges()const{
    return boost::edges(g_);
}


template<typename VertexData,typename EdgeData>
BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode::SerialGraphNode(const typename BoostSerialGraph<VertexData,EdgeData>::vertex_data_t& data_)
:
data(data_),
id(bitrl::consts::INVALID_ID)
{}

template<typename VertexData,typename EdgeData>
BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode::SerialGraphNode(typename BoostSerialGraph<VertexData,EdgeData>::vertex_data_t&& data_)
:
data(data_),
id(bitrl::consts::INVALID_ID)
{
    data_.clear();
}

template<typename VertexData,typename EdgeData>
BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode::SerialGraphNode(const typename BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode& o)
:
data(o.data),
id(o.id)
{}


template<typename VertexData,typename EdgeData>
typename BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode&
BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode::operator=(const typename BoostSerialGraph<VertexData,EdgeData>::SerialGraphNode& o){

    if(this == &o)
        return *this;

    this->data = o.data;
    this->id = o.id;
    return *this;

}

/*template<typename VertexData,typename EdgeData>
bool operator==(const typename boost_unidirected_serial_graph<VertexData,EdgeData>::vertex_t& v1,
                const typename boost_unidirected_serial_graph<VertexData,EdgeData>::vertex_t& v2){

    return (v2.id == v1.id);
}

template<typename VertexData,typename EdgeData>
bool operator!=(const typename boost_unidirected_serial_graph<VertexData,EdgeData>::vertex_t& v1,
                const typename boost_unidirected_serial_graph<VertexData,EdgeData>::vertex_t& v2){

    return !(v2 == v1);
}*/

/*template<typename VertexData>
inline
bool operator==(const typename boost_unidirected_serial_graph<VertexData,void>::vertex_t& v1,
                const typename boost_unidirected_serial_graph<VertexData,void>::vertex_t& v2){

    return (v2.id == v1.id);
}

template<typename VertexData>
inline
bool operator!=(const typename boost_unidirected_serial_graph<VertexData,void>::vertex_t& v1,
                const typename boost_unidirected_serial_graph<VertexData,void>::vertex_t& v2){

    return !(v2 == v1);
}*/


}
}
#endif // BOOST_SERIAL_GRAPH_H
