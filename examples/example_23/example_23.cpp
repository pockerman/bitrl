/**
  * A* search on a road network. This example
  * requires that the OSMNX (https://osmnx.readthedocs.io/)
  * library is installed as well as the NetworkX (https://networkx.github.io/documentation/stable/)
  * library. The example is taken from the Autonomous Vehicle course on Coursera
  *
 */


#include "cubeai/base/cubeai_types.h"
#include "cubeai/base/math_constants.h"

#include <boost/python.hpp>

#include <vector>
#include <map>
#include <queue>
#include <set>
#include <iostream>
#include <limits>
#include <cmath>
#include <exception>
#include <algorithm>


namespace example{

typedef boost::python::api::object obj_t;
using cubeai::real_t;
using cubeai::uint_t;

template<typename IdTp>
std::vector<IdTp>
reconstruct_a_star_path(const std::multimap<IdTp, IdTp>& map, const IdTp& start){

    if(map.empty()){
        return std::vector<IdTp>();
    }

    std::vector<IdTp> path;
    path.push_back(start);

    auto next_itr = map.find(start);

    if(next_itr == map.end()){

        //such a key does not exist
        throw std::logic_error("Key: "+std::to_string(start)+" does not exist");
    }

    IdTp next = next_itr->second;
    path.push_back(next);

    while(next_itr!=map.end()){

        next_itr = map.find(next);
        if(next_itr != map.end()){
            next = next_itr->second;
            path.push_back(next);
        }
    }

    //let's reverse the path
    std::vector<IdTp> the_path;
    the_path.reserve(path.size());
    auto itrb = path.rbegin();
    auto itre = path.rend();

    while(itrb != itre){
        the_path.push_back(*itrb++);
    }

    return the_path;
}


template<
    class T,
    class Container = std::vector<T>,
    class Compare = std::less<typename Container::value_type> >
class searchable_priority_queue: public std::priority_queue<T, Container, Compare>
{

public:

    using std::priority_queue<T, Container, Compare>::priority_queue;

   /// \brief Returns true if the given value is contained
   /// internally it calls std::find.
   bool contains(const T& val)const;
};

template<typename T,typename Container,typename Compare>
bool
searchable_priority_queue<T,Container,Compare>::contains(const T& val)const{

    auto itr = std::find_if(this->c.cbegin(), this->c.cend(), [&](const auto& node){
        if(node.id == val.id){
            return true;
        }

        return false;
    });

   if(itr == this->c.end())
   {
        return false;
   }

   return true;
}


struct fcost_comparison
{
   template<typename NodeTp>
   bool operator()(const NodeTp& n1,const NodeTp& n2)const;
};

template<typename NodeTp>
bool
fcost_comparison::operator()(const NodeTp& n1,const NodeTp& n2)const{

   if(n1.f_cost > n2.f_cost){
       return true;
   }

   return false;
}

struct id_comparison
{
   template<typename NodeTp>
   bool operator()(const NodeTp& n1,const NodeTp& n2)const;
};

template<typename NodeTp>
bool
id_comparison::operator()(const NodeTp& n1,const NodeTp& n2)const{

   if(n1.id > n2.id){
       return true;
   }

   return false;
}

struct Node
{

    uint_t id;
    real_t x;
    real_t y;
    uint_t street_count;

    // holds the heuristic cost for the node
    real_t f_cost{0.0};
    real_t g_cost{0.0};
};

bool operator==(const Node& n1, const Node& n2){
    return n1.id == n2.id;
}

struct Edge
{
    uint_t in_vertex;
    uint_t out_vertex;
    real_t length;
};

// the heuristic to use for A*
struct DistanceHeuristic
{

    //
    real_t operator()(const Node& n1, const Node& n2)const;

};

real_t
DistanceHeuristic::operator()(const Node& n1, const Node& n2)const{

    auto long1 = n1.x*cubeai::MathConsts::PI/180.0;
    auto lat1  = n1.y*cubeai::MathConsts::PI/180.0;
    auto long2 = n2.x*cubeai::MathConsts::PI/180.0;
    auto lat2  = n2.y*cubeai::MathConsts::PI/180.0;

    // Use a spherical approximation of the earth for
    // estimating the distance between two points.
    auto r = 6371000;
    auto x1 = r*std::cos(lat1)*std::cos(long1);
    auto y1 = r*std::cos(lat1)*std::sin(long1);
    auto z1 = r*std::sin(lat1);

    auto x2 = r*std::cos(lat2)*std::cos(long2);
    auto y2 = r*std::cos(lat2)*std::sin(long2);
    auto z2 = r*std::sin(lat2);

    auto d = std::pow(x2-x1, 2) + std::pow(y2-y1, 2) + std::pow(z2-z1, 2);
    return std::sqrt(d);
}

// wrapper to facilitate the Python API
class Graph
{
public:

    typedef Node vertex_type;

    // constructor
    Graph(std::string start, std::string end, obj_t py_namespace);

    // build the OSNX graph
    void build();

    // number of nodes
    uint_t n_nodes()const noexcept{return nodes_.size();}

    uint_t start_node()const{return start_node_.id;}

    // returns the edge associated with the given node
    std::vector<Edge> get_node_edges(const Node node)const;

    // get the node with the given idx
    Node& get_node(uint_t idx);

    // build the A* path
    std::multimap<uint_t, uint_t> get_a_star_path();

private:

    std::string start_;
    std::string end_;
    obj_t py_namespace_;
    obj_t map_graph_;

    Node start_node_;
    Node end_node_;

    std::vector<Node> nodes_;

    // build the nodes list
    void build_nodes_list_();

};

Graph::Graph(std::string start, std::string end, obj_t py_namespace)
    :
      start_(start),
      end_(end),
      py_namespace_(py_namespace),
      start_node_(),
      end_node_()
{}

void
Graph::build_nodes_list_(){

    std::string nodes_list_str = "node_data = map_graph.nodes(True)\n";
    nodes_list_str += "node_data = list(node_data)\n";
    boost::python::exec(nodes_list_str.c_str(), py_namespace_);

    auto node_data = boost::python::extract<boost::python::list>(py_namespace_["node_data"]);

    nodes_.reserve(boost::python::len(node_data));

    for(auto i=0; i<boost::python::len(node_data); ++i){

        Node n;
        auto node_tuple = boost::python::extract<boost::python::tuple>(node_data()[i]);

        n.id = boost::python::extract<uint_t>(node_tuple()[0]);
        n.x = boost::python::extract<real_t>(node_tuple()[1]["x"]);
        n.y = boost::python::extract<real_t>(node_tuple()[1]["y"]);
        n.street_count = n.x = boost::python::extract<uint_t>(node_tuple()[1]["street_count"]);
        nodes_.push_back(n);
    }
}

// get the node with the given idx
Node&
Graph::get_node(uint_t idx){

    for(auto& node : nodes_){
        if(node.id == idx){
            return node;
        }
    }


    throw std::logic_error("Invalid node id");
}

std::vector<Edge>
Graph::get_node_edges(const Node node)const{

     std::string edges_list_str = "edges = map_graph.out_edges([" + std::to_string(node.id) + "], data=True)\n";
     edges_list_str += "edges = list(edges))\n";
     boost::python::exec(edges_list_str.c_str(), py_namespace_);

     std::vector<Edge> edges;
     auto edges_list = boost::python::extract<boost::python::list>(py_namespace_["edges"]);

     edges.reserve(boost::python::len(edges_list));

     for(auto i=0; i<boost::python::len(edges_list); ++i){

         Edge e;
         auto edge_tuple = boost::python::extract<boost::python::tuple>(edges_list()[i]);

         e.in_vertex  = boost::python::extract<uint_t>(edge_tuple()[0]);
         e.out_vertex = boost::python::extract<uint_t>(edge_tuple()[1]);
         e.length = boost::python::extract<real_t>(edge_tuple()[2]["length"]);

         edges.push_back(e);
     }

     return edges;

}

void
Graph::build(){

    std::string imports = "import osmnx as ox\n";
    auto ignore = boost::python::exec(imports.c_str(), py_namespace_);

    std::string map_graph_str = "map_graph = ox.graph_from_place('" + start_ + "," + end_ + "'," + "network_type='drive')\n";

    // fill in the map graph
    boost::python::exec(map_graph_str.c_str(), py_namespace_);

    std::string origin_str = "origin = ox.get_nearest_node(map_graph, (37.8743, -122.277))\n";
    boost::python::exec(origin_str.c_str(), py_namespace_);

    uint_t start_node_id = boost::python::extract<uint_t>(py_namespace_["origin"]);
    std::string destination_str = "destination = list(map_graph.nodes())[-1]";

    boost::python::exec(destination_str.c_str(), py_namespace_);
    uint_t end_node_id = boost::python::extract<uint_t>(py_namespace_["destination"]);

    start_node_.id = start_node_id;
    end_node_.id = end_node_id;

    build_nodes_list_();

    auto& start_node = get_node(start_node_id);
    start_node_ = start_node;

    auto& end_node = get_node(end_node_id);
    end_node_ = end_node;
}

std::multimap<uint_t, uint_t>
Graph::get_a_star_path(){

    // for each node hodls where it
    // came form
    std::multimap<uint_t, uint_t> came_from;

    // start and goal node are the same
    if(start_node_ == end_node_){
        came_from.insert({start_node_.id, start_node_.id});
        return came_from;
    }

    // the object to use for the
    DistanceHeuristic dist_h;

    searchable_priority_queue<vertex_type, std::vector<vertex_type>, fcost_comparison> open;

    // set of explored vertices
    std::set<uint_t> explored;

    //the cost of the path so far leading to this
    //node is obviously zero at the start node
    start_node_.g_cost = 0.0;

    //calculate the fCost from start node to the goal
    //at the moment this can be done only heuristically
    //start.data.fcost = h(start.data.position, end.data.position);
    start_node_.f_cost = dist_h(start_node_, end_node_);
    open.push(start_node_);

    bool goal_found = true;
    while(!open.empty()){

       //the vertex currently examined
       const vertex_type cv = open.top();
       open.pop();

       //check if this is the goal
       if(cv == end_node_){
          goal_found = true;
          break;
       }

       // the cost for the cv
       auto cv_g_cost = cv.g_cost;

       //get the adjacent neighbors
       auto edges = get_node_edges(cv);
       auto edge_itr_begin = edges.begin();
       auto edge_itr_end = edges.end();

       // loop over the neighbors
       for(; edge_itr_begin != edge_itr_end; ++edge_itr_begin){

          auto& edge = *edge_itr_begin;

          auto out_vertex = edge.out_vertex;

          if(explored.contains(out_vertex)){
              continue;
          }

          // get the node associated with the
          // out vertex
          auto& node = get_node(out_vertex);

          // what is the length of the edge
          auto edge_length = edge.length;

          // if the out vertex is not in the
          // open set
          if(!open.contains(node)){

              // the cost for the out vertex
              node.g_cost = cv_g_cost + edge_length;
              node.f_cost = cv_g_cost + edge_length + dist_h( node, end_node_);
              open.push(node);
              came_from.insert({out_vertex, cv.id});
          }
          else{

              auto v_cost = node.g_cost;

              if(cv_g_cost + edge_length < v_cost ){
                    node.g_cost = cv_g_cost + edge_length;
                    node.f_cost = cv_g_cost + edge_length + dist_h( node, end_node_);
                    came_from.insert({out_vertex, cv.id});
              }
          }
       }

       //current node is not the goal so proceed
       //add it to the explored (or else called closed) set
       explored.insert(cv.id);

    }

    if(!goal_found){
        throw std::logic_error("Goal not found");
    }

    return came_from;

}

}


int main(){

    using namespace example;

    try{

        Py_Initialize();
        auto main_module = boost::python::import("__main__");
        auto main_namespace = main_module.attr("__dict__");

        // create the graph
        Graph graph("Berkeley", "California", main_namespace);
        graph.build();

        std::cout<<"Number of nodes="<<graph.n_nodes()<<std::endl;

        auto path_map = graph.get_a_star_path();
        auto path = reconstruct_a_star_path(path_map, graph.start_node());
    }
    catch(const boost::python::error_already_set&)
    {
            PyErr_Print();
    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){

        std::cout<<"Unknown exception occured"<<std::endl;
    }
    return 0;
}
