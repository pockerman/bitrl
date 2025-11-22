#include "bitrl/bitrl_consts.h"

#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/geometry/mesh/mesh_connectivity.h"
#include "bitrl/utils/geometry/mesh/quad_mesh_generation.h"
#include "bitrl/utils/geometry/mesh/mesh.h"
#include "bitrl/utils/geometry/mesh/element.h"
#include "bitrl/utils/geometry/mesh/node.h"
#include "bitrl/utils/geometry/mesh/face_element.h"
#include "bitrl/utils/geometry/mesh/face_mesh_iterator.h"
#include "bitrl/utils/geometry/mesh/element_mesh_iterator.h"
#include "bitrl/utils/geometry/mesh/element_type.h"

#include "bitrl/utils/geometry/mesh/mesh_predicates.h"
#include "bitrl/utils//predicates.h"

#include <exception>
#include <vector>

namespace bitrl{
namespace utils{
namespace geom{
	
namespace{
 inline
 uint_t idx(uint_t nx,
               uint_t i,
               uint_t j)
 {
        return i+j*(nx+1);
 }


uint_t
find_neighbor_id(const Element<2>& e, uint_t f,
                     const std::vector<MeshConnectivity>& node_connections)
{

   std::vector<uint_t> face_vertices;

   e.face_vertices(f,face_vertices);

   //get the connectivity of vertex 0
   MeshConnectivity v0(node_connections[face_vertices[0]]);

   //get the connectivity of vertex 1
   MeshConnectivity v1(node_connections[face_vertices[1]]);

   //sort the two connectivities
   std::sort(v0.connectivity_begin(),v0.connectivity_end());
   std::sort(v1.connectivity_begin(),v1.connectivity_end());

   std::vector<uint_t> common;

   //compute the intersection
   std::set_intersection(v0.connectivity_begin(),v0.connectivity_end(),
                         v1.connectivity_begin(),v1.connectivity_end(),
						 std::back_inserter(common));

   //remove the id of the element we work on
   common.erase(std::remove(common.begin(), common.end(),
                            e.get_id()),common.end());

   if(common.empty()){
       return bitrl::consts::INVALID_ID;
   }

   //if we are not empty return the first
   return common[0];
}


 void build_rectangular_mesh_vertices(const GeomPoint<2>& lower_point,
                                       const GeomPoint<2>& upper_point,
                                       uint_t Nx,uint_t Ny, Mesh<2>& mesh)
 {


   const GeomPoint<2>& p1 = lower_point;
   const GeomPoint<2>& p2 = upper_point;

   const real_t dx = (p2[0]-p1[0])/static_cast<real_t>(Nx);
   const real_t dy = (p2[1]-p1[1])/static_cast<real_t>(Ny);

   //dummy variable for counting the vertices in the mesh
   uint_t node_id = 0;

   //construct the vertices of the mesh
   for(uint_t j=0; j<=Ny; ++j)
   {

     for(uint_t i=0; i<=Nx; ++i)
     {
       real_t data[] = {p1[0]+i*dx, p1[1]+j*dy};
       mesh.create_vertex(GeomPoint<2>(data), node_id++);
     }
   }

 }

void build_rectangular_mesh_elements(uint_t Nx, uint_t Ny, uint_t n_nodes_per_elem,
                                       Mesh<2>& mesh, std::vector<MeshConnectivity>& vertex_connections){


    //dummy variable for the id of the element
    uint_t elem_id = 0;

    //build the elements
     for (uint_t j=0; j<Ny; j++)
     {
      for(uint_t i = 0; i<Nx; i++)
       {

          //build the element
          auto* element = mesh.create_element(ElementType::sub_type::QUAD4, elem_id++, 0);

          if(!element){
              throw std::logic_error("NULL pointer element");
          }

          element->resize_nodes();
          uint_t node_id = idx(Nx, i, j);

          auto* node = mesh.node(node_id);
          element->set_node(0, node);
          vertex_connections[node_id].push_back(element->get_id());

          node_id = idx(Nx,i+1,j);
          node = mesh.node(node_id);
          element->set_node(1, node);
          vertex_connections[node_id].push_back(element->get_id());

          node_id = idx(Nx,i+1,j+1);
          node = mesh.node(node_id);
          element->set_node(2, node);
          vertex_connections[node_id].push_back(element->get_id());

          node_id = idx(Nx,i,j+1);
          node = mesh.node(node_id);
          element->set_node(3, node);
          vertex_connections[node_id].push_back(element->get_id());
       }
   }
}

 void build_quad_mesh_element_element_connectivity(Mesh<2>& m,
                                                  const std::vector<MeshConnectivity>& vertices_connections){

    ElementMeshIterator<Active, Mesh<2>> iterator(m);

    auto e_it = iterator.begin();
    auto e_it_end = iterator.end();

    for(; e_it != e_it_end; ++e_it){

      Element<2>* element = *e_it;
      element->resize_neighbors();

      //the neighbors of the current element
      MeshConnectivity neigh_connections(element->n_faces());

      //loop over the faces of the element
      for(uint_t face=0; face<element->n_faces(); ++face)
      {

         // find the id of the neighbor
         uint_t neigh_id = find_neighbor_id(*element, face, vertices_connections);

         if(neigh_id == bitrl::consts::INVALID_ID){
             element->set_neighbor(face, nullptr);
         }
         else{
             element->set_neighbor(face, m.element(neigh_id));
         }
      }
    }
}

void build_quad_mesh_element_face_connectivity(Mesh<2>& m){

  ElementMeshIterator<Active, Mesh<2>> iterator(m);

  auto e_it = iterator.begin();
  auto e_it_end = iterator.end();

  uint_t face_counter = 0;

  for(; e_it!=e_it_end; ++e_it){

    Element<2>* element = *e_it;
    element->resize_faces();

    // loop over the neighbors
    for(uint_t neigh=0; neigh<element->n_neighbors(); ++neigh){

        if(element->neighbor_ptr(neigh) == nullptr){

            //this is a boundary face
            FaceElement<2,1>* face = m.create_face(ElementType::sub_type::EDGE2, face_counter++);

            //by default all boundary faces get an indicator zero
            face->set_boundary_indicator(0);
            face->set_owner_element(element);
            face->resize_nodes();

            std::vector<uint_t> face_vertices;

            //get the vertices of the element at the given face
            element->face_vertices(neigh, face_vertices);

            // assign the vertices of the face
            for(uint_t v=0; v<face_vertices.size(); ++v){

                auto* vertex = m.node(face_vertices[v]);
                face->set_node(v, vertex);
            }

            // set the face of the element
            element->set_face(neigh, face);
        }
        else{
            //this is an internal face. Create the face
            //only if our id is smaller than our neighbor
            if(element->get_id() < element->neighbor_ptr(neigh)->get_id()){

               FaceElement<2,1>* face = m.create_face(ElementType::sub_type::EDGE2,face_counter++);

               Element<2>* neighbor = element->neighbor_ptr(neigh);

               face->set_owner_element(element);
               face->set_shared_element(neighbor);
               face->resize_nodes();

               std::vector<uint_t> face_vertices;

              //get the vertices of the element at the given face
              element->face_vertices(neigh,face_vertices);

              // assign the vertices of the face
              for(uint_t v=0; v<face_vertices.size(); ++v){

                  auto* vertex = m.node(face_vertices[v]);
                  face->set_node(v, vertex);
              }

              // set the face of the element
              element->set_face(neigh, face);

              //find out which local neighbor the current element is
              //to our neighbor
              uint_t neigh_idx = neighbor->which_neighbor_am_i(*element);

              if(neigh_idx == bitrl::consts::INVALID_ID){
                  throw std::logic_error("Invalid neighbor index");
              }

              neighbor->set_face(neigh_idx, face);
            }
        }
    }
   }
}

}

void build_quad_mesh(Mesh<2>& mesh, uint_t nx, uint_t ny,
                     const GeomPoint<2, real_t>& lower_point,
                     const GeomPoint<2, real_t>& upper_point){


    const uint_t Nx = nx;
    const uint_t Ny = ny;

    //
    mesh.reserve_n_nodes((Nx+1)*(Ny+1));

    //build the vertices of the mesh
    build_rectangular_mesh_vertices(lower_point, upper_point,Nx, Ny, mesh);

    //now that we have the vertices lets build the elements
    //reserve space for the elements
    mesh.reserve_n_elements(Nx*Ny);

    std::vector<MeshConnectivity> vertices_connections(mesh.n_vertices(), MeshConnectivity());

    //TODO: change hard-coded for QUAD4 here!!!
    build_rectangular_mesh_elements(Nx,Ny, 4, mesh, vertices_connections);

    //now we build the neighbors of the mesh
    build_quad_mesh_element_element_connectivity(mesh, vertices_connections);

    //count how many faces the mesh has
    uint_t n_internal_faces=0;
    uint_t n_boundary_faces=0;

    ElementMeshIterator<Active, Mesh<2>> iterator(mesh);

    auto e_it = iterator.begin();
    auto e_it_end = iterator.end();

    for(; e_it!=e_it_end; ++e_it){

       Element<2>* element = *e_it;

       for(uint_t neigh=0; neigh < element->n_neighbors(); ++neigh){

           if(element->neighbor_ptr(neigh) == nullptr){
               n_boundary_faces++;
           }
           else{
              // an internal face is shared
              // so this is counted twice
              n_internal_faces++;
           }
       }

       //we want to create the faces of the element
       element->resize_faces();
    }

    mesh.reserve_n_faces(n_boundary_faces + n_internal_faces/2);
    build_quad_mesh_element_face_connectivity(mesh);

    //finally set the number of boundaries
    mesh.set_n_boundaries(4);

    FaceMeshIterator<ActiveBoundaryObject, Mesh<2>>filter(mesh);

    auto face_begin = filter.begin();
    auto face_end   = filter.end();

    for(; face_begin != face_end; ++face_begin){

        auto* face = *face_begin;

        const GeomPoint<2> centroid = face->centroid();

        if(centroid[1] == lower_point[1]){
           face->set_boundary_indicator(0);
        }
        else if(centroid[0] == upper_point[0]){
          face->set_boundary_indicator(1);
        }
        else if(centroid[1]==upper_point[1]){
          face->set_boundary_indicator(2);
       }
       else if(centroid[0] == lower_point[0]){
         face->set_boundary_indicator(3);
       }
     }
}

}

}
}
