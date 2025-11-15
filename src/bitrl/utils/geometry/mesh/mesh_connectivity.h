#ifndef MESH_CONNECTIVITY_H
#define MESH_CONNECTIVITY_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/bitrl_consts.h"

#include<ostream>
#include<vector>
#include<iterator>

namespace bitrl{
namespace utils{
namespace geom{

/// \brief MeshConnectivity class stores the various connectivities
/// for a mesh object
class MeshConnectivity
{

public:
 
    typedef std::vector<uint_t>::iterator connectivity_iterator;
    typedef std::vector<uint_t>::const_iterator const_connectivity_iterator;

    MeshConnectivity();

    /**
      *\detailed ctor construct by setting the size of the
      * connectivity
      */
    explicit MeshConnectivity(uint_t n);

    /**
      *\detailed cto construct by passing the std::vector that holds the
      *global connection indices
      */
    explicit MeshConnectivity(const std::vector<uint_t>& conn);

    /**
      *\detailed copy constructor
      */
    MeshConnectivity(const MeshConnectivity& o);

    /**
      *\detailed copy assignement operator
      */
    MeshConnectivity& operator=(const MeshConnectivity& o);

    /**
      *\detailed assign the global indices of the MeshEntities
      * that the MeshEntity this MeshConnectivity corresponds to
      */
    void assign_connections(const std::vector<uint_t>& connections);

    /**
      *\detailed set the size of the connectivity
      */
    void set_connectivity_size(uint_t size){connections_.resize(size);}

    /**
      *\detailed set the connection at position \p i to \p connection_idx
      */
    void set_connection(uint_t i, uint_t connection_idx){connections_[i] =connection_idx;}

    /**
      *\detailed invalidate the connections
      */
    void invalidate_connections();

    /**
      *\detailed clear the connections returns to a state just like when calling the
      *default ctor
      */
    void clear_connections(){connections_.clear();}

    /**
      *\detailed add to the end the given idx
      */
    void push_back(uint_t idx){connections_.push_back(idx);}

    /**
      *\detailed get the i-th connectivity index
      */
    uint_t operator[](uint_t i)const{return connections_[i];}

    /**
      *\detailed get the size for this connectivity
      */
    uint_t size()const{return connections_.size();}

    /**
      *\detailed true iff the \p connections_.empty() returs \p true
      */
    bool empty()const{return connections_.empty();}


    /**
      *\detailed get the connection for the local index i
      */
    uint_t connection_idx(uint_t i)const;


    /**
      *\detailed print the connectivity info
      */
    std::ostream& print_connectivity_info(std::ostream& o)const;

    connectivity_iterator connectivity_begin(){return connections_.begin();}
    connectivity_iterator connectivity_end(){return connections_.end();}

    const_connectivity_iterator connectivity_begin()const{return connections_.begin();}
    const_connectivity_iterator connectivity_end()const{return connections_.end();}


private:
 
 
     /**
       *\detailed the global indices of the MeshEntities
       *that the MeshEntity this MeshConnectivity corresponds to
       */
     std::vector<uint_t> connections_;

};


inline
MeshConnectivity::MeshConnectivity()
    :
      connections_()
{}

inline
MeshConnectivity::MeshConnectivity(uint_t n)
    :
      connections_(n, bitrl::consts::INVALID_ID)
{}

inline
MeshConnectivity::MeshConnectivity(const std::vector<uint_t>& conn)
    :
      connections_(conn)
{}

inline
MeshConnectivity::MeshConnectivity(const MeshConnectivity& o)
                 :
                 connections_(o.connections_)
                 {}
inline
MeshConnectivity& 
MeshConnectivity::operator=(const MeshConnectivity& o)
{

  if(this == &o){
      return *this;
  }
  
  connections_.assign(o.connections_.begin(),o.connections_.end());
  return *this;
}

inline
void
MeshConnectivity::assign_connections(const std::vector<uint_t>& connections){

  connections_.assign(connections.begin(),connections.end());
}

inline
std::ostream& 
MeshConnectivity::print_connectivity_info(std::ostream& out)const{
  std::copy(connections_.begin(),connections_.end(),std::ostream_iterator<uint_t>(out,"\n"));
  return out;

}

inline
uint_t
MeshConnectivity::connection_idx(uint_t i)const
{
    if(i >= connections_.size()){
        throw std::logic_error("Invalid connection index: "+std::to_string(i) +
                               " not in [0, "+
                               std::to_string(connections_.size()));
    }

  return connections_[i];
}

inline
std::ostream& operator<< (std::ostream &out, const MeshConnectivity& connectivity){

  return connectivity.print_connectivity_info(out);

}

}

}
}
#endif
