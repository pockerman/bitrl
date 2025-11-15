#include "bitrl/utils/geometry/mesh/face_element.h"
#include "bitrl/utils/geometry/mesh/element.h"
//#include "kernel/discretization/face.h"
#include "bitrl/utils/geometry/mesh/node.h"
//#include "kernel/base/kernel_consts.h"

namespace bitrl{
namespace utils{
namespace geom{
	
template<int spacedim>
FaceElement<spacedim,0>::FaceElement()
                        :
                        GeomPoint<spacedim>(),
						MeshEntity(),
                        is_vertex_(false)
                        {}
               
    
template<int spacedim>
FaceElement<spacedim,0>::FaceElement(uint_t global_id,
                                     real_t val,
                                     uint_t pid,
									 const std::any& data)
                                     :
                                     GeomPoint<spacedim>(val),
									 MeshEntity(global_id, pid, data),
                                     is_vertex_(false)
                                     {}

template<int spacedim>
FaceElement<spacedim,0>::FaceElement(const std::array<real_t, spacedim>& coords,
                                     uint_t global_id,
                                     uint_t pid,
									 const std::any& data)
                                     :
                                     GeomPoint<spacedim>(coords),
									 MeshEntity(global_id, pid, data),
                                     is_vertex_(false)
                                     {}
                

template<int spacedim>
FaceElement<spacedim,0>::FaceElement(const GeomPoint<spacedim>& point, 
                                     uint_t global_id,
                                     uint_t pid,
									 const std::any& data)
                                     :
                                     GeomPoint<spacedim>(point),
									 MeshEntity(global_id, pid, data),
                                     is_vertex_(false)
                                     {}
                
template<int spacedim>
FaceElement<spacedim,0>::FaceElement(const FaceElement<spacedim,0>& t)
                        :
                       GeomPoint<spacedim>(t.coordinates()),                       
                       is_vertex_(t.is_vertex_)
                       {}

template<int spacedim>
FaceElement<spacedim,0>& 
FaceElement<spacedim,0>::operator=(const FaceElement<spacedim,0>& o){

  if(this==&o)return *this;
  
  this->GeomPoint<spacedim>::operator=(o);
  is_vertex_ = o.is_vertex_;
  this->set_id(o.get_id());
  return *this;

}

	
FaceElement<2,1>::FaceElement()
                        :
                        owner_(nullptr),
                        neighbor_(nullptr),
                        boundary_indicator_(bitrl::consts::INVALID_ID),
                        n_nodes_(2)
                        {}
                        
FaceElement<2,1>::FaceElement(uint_t id,uint_t pid)
                 :

                 owner_(nullptr),
                 neighbor_(nullptr),
                 boundary_indicator_(bitrl::consts::INVALID_ID),
                 n_nodes_(2)
                 {}

FaceElement<2,1>::FaceElement(uint_t id, uint_t nnodes, uint_t pid)
                 :
                 owner_(nullptr),
                 neighbor_(nullptr),
                 boundary_indicator_(bitrl::consts::INVALID_ID),
                 n_nodes_(nnodes)
                 {}
                 
std::ostream& 
FaceElement<2,1>::print_(std::ostream& out)const
{
  out<<" Boundary Indicator= "<<boundary_indicator_<<std::endl;
  
  if(owner_){
    out<<" Owner Id= "<<owner_->get_id()<<std::endl;
  }
  
  if(neighbor_){
    out<<" Neighbor Id= "<<neighbor_->get_id()<<std::endl;
  }
  
  return out;

}

void
FaceElement<2,1>::resize_nodes(){
    this->nodes_.resize(n_nodes_, nullptr);
}

FaceElement<2,1>::node_ptr_t
FaceElement<2,1>::node_ptr(uint_t n){
    if(n >= n_nodes()){
        throw std::logic_error("Invalid node index: " +
                               std::to_string(n) +
                               " not in [0," +
                               std::to_string(n_nodes()) +
                               ")");
    }

    return nodes_[n];
}

void
FaceElement<2,1>::set_node(uint_t n, FaceElement<2,1>::node_ptr_t node){

    if(n >= n_nodes()){
        throw std::logic_error("Invalid node index: " +
                               std::to_string(n) +
                               " not in [0," +
                               std::to_string(n_nodes()) +
                               ")");
    }

    if(this->nodes_.empty()){
        throw std::logic_error("Nodes list has not been initialized");
    }

    this->nodes_[n] = node;

}

std::vector<uint_t>
FaceElement<2,1>::get_vertices_ids()const{

    return {nodes_[0]->get_id(), nodes_[1]->get_id()};
}


FaceElement<2,1>::cnode_ref_t
FaceElement<2,1>::get_vertex(uint_t v)const{

    if(v >= n_vertices()){
        throw std::logic_error("Invalid vertex index: " +
                               std::to_string(v) +
                               " not in [0," +
                               std::to_string(n_vertices()) +
                               ")");
    }

    if(nodes_.empty()){
        throw std::logic_error("Nodes list has not been populated");
    }


    return *nodes_[v];
}


FaceElement<2,1>::node_ref_t
FaceElement<2,1>::get_vertex(uint_t v){

    if(v >= n_vertices()){
        throw std::logic_error("Invalid vertex index: " +
                               std::to_string(v) +
                               " not in [0," +
                               std::to_string(n_vertices()) +
                               ")");
    }

    if(nodes_.empty()){
        throw std::logic_error("Nodes list has not been populated");
    }


    return *nodes_[v];

}

real_t
FaceElement<2,1>::owner_neighbor_distance()const{

    if(!owner_){
        throw std::logic_error("FaceElement owner has not been set");
    }

    if(!neighbor_){

        return owner_->centroid().distance(centroid());
    }

    return owner_->centroid().distance(neighbor_->centroid());
}

real_t
FaceElement<2,1>::volume()const{

    if(this->nodes_.empty()){
        throw std::logic_error("Nodes list has not been initialized");
    }

    return (*nodes_[0]).distance(*nodes_[1]);
}

const GeomPoint<2>
FaceElement<2,1>::centroid()const{

    if(this->nodes_.empty()){
        throw std::logic_error("Nodes list has not been initialized");
    }

    real_t x = 0.5*((*this->nodes_[1])[0] +(*this->nodes_[0])[0]);
    real_t y = 0.5*((*this->nodes_[1])[1] + (*this->nodes_[0])[1]);
    std::vector<real_t> coords{x ,y};
    return GeomPoint<2>(coords);
}



bool
FaceElement<2,1>::is_owner(uint_t id)const{

    if(!owner_ ){
        throw std::logic_error("Owner pointer is NULL");
    }

    return owner_->get_id() == id;
}

Element<2>&
FaceElement<2,1>::get_neighbor(){

    if(!neighbor_){
        throw std::logic_error("NULL neighbor pointer requested");
    }

    return *neighbor_;
}

const Element<2>&
FaceElement<2,1>::get_neighbor()const{

    if(!neighbor_){
        throw std::logic_error("NULL neighbor pointer requested");
    }

    return *neighbor_;

}



Element<2>&
FaceElement<2,1>::get_owner(){

    if(!owner_){
        throw std::logic_error("NULL owner pointer requested");
    }

    return *owner_;
}

const Element<2>&
FaceElement<2,1>::get_owner()const{
    if(!owner_){
        throw std::logic_error("NULL owner pointer requested");
    }

    return *owner_;
}

const DynVec<real_t>
FaceElement<2,1>::normal_vector()const{

    if(!owner_){
        throw std::logic_error("NULL owner pointer requested");
    }

    uint_t fidx = owner_->which_face_am_i(*this);
    return owner_->face_normal_vector(fidx);

}
	
	
	template class FaceElement<1,0>;
	template class FaceElement<2,0>;
}
}
}
