#ifndef GENERIC_LINE_H
#define GENERIC_LINE_H

#include "bitrl/bitrl_types_v2.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/maths/math_utils.h"

#include <cmath>
#include <stdexcept>

namespace bitrl{
namespace utils{
namespace geom{	



///
/// \brief class GenericLine. Represents a generic line
/// with vertex VertexType
/// The equation of a linein l=Ax + By + C
///
template<int dim>
class GenericLine
{
public:

    static const int dimension = dim;
    
    typedef GeomPoint<dim> vertex_type;

    ///
    /// \brief Constructor
    ///
    GenericLine();

    ///
    /// \brief Constructor
    ///
    GenericLine(const vertex_type& p1, const vertex_type& p2);

    ///
    /// \brief Constructor
    ///
    GenericLine(const vertex_type& p1, const vertex_type& p2, uint_t id);

    ///
    /// \brief Returns the v-th vertex of the segment
    ///
    const vertex_type& get_vertex(uint_t v)const;

    ///
    /// \brief Returns the v-th vertex of the segment
    ///
    vertex_type& get_vertex(uint_t v);
	
	///
	/// \brief Returns true iff id_ != rlenvscpp::consts::INVALID_ID
	///
    bool has_valid_id()const noexcept{return id_ != bitrl::consts::INVALID_ID;}
	
	///
    /// \brief Returns the id of the line
	///
    uint_t get_id()const noexcept{return id_;}
	
	///
    /// \brief Set the id of the line
	///
    void set_id(uint_t id)noexcept{id_ = id;}
	
	///
	/// \brief The slope of the line
	///
	real_t slope()const;
	
	
	///
	/// \brief Returns the constant factor
	///
	real_t factor()const;
	
	///
	/// \brief Returns the perpendicular distance from the
	/// line to the node
	///
	real_t distance(const vertex_type& n)const;
	
	///
	/// \brief Calculate the length of the line
	///
	real_t length()const{return end_.distance(start_);}
	
private:
	
	uint_t id_;
	vertex_type start_;
	vertex_type end_;
		
};

template<int dim>
GenericLine<dim>::GenericLine()
:
id_(bitrl::consts::INVALID_ID),
start_(),
end_()
{}

template<int dim>
GenericLine<dim>::GenericLine(const vertex_type& p1, const vertex_type& p2, uint_t id)
:
id_(id),
start_(p1),
end_(p2)
{}

template<int dim>
GenericLine<dim>::GenericLine(const vertex_type& p1, const vertex_type& p2)
:
GenericLine<dim>(p1,p2,bitrl::consts::INVALID_ID)
{}

template<int dim>
const typename GenericLine<dim>::vertex_type&
GenericLine<dim>::get_vertex(uint_t v)const{

    if(v == 0){
        return start_;
    }
    else if(v == 1){
        return end_;
    }

    throw std::logic_error("Invalid vertex index. Index not in [0,1]");

}

template<int dim>
typename GenericLine<dim>::vertex_type&
GenericLine<dim>::get_vertex(uint_t v){

    if(v == 0){
        return start_;
    }
    else if(v == 1){
        return end_;
    }

    throw std::logic_error("Invalid vertex index. Index not in [0,1]");

}

template<int dim>
real_t 
GenericLine<dim>::slope()const{
	return (end_[1] - start_[1])/(end_[0] - start_[0]);
}

template<int dim>
real_t 
GenericLine<dim>::factor()const{
	auto slope_  = slope();
	return start_[1] - slope_ * start_[0];
}

template<>
real_t 
GenericLine<2>::distance(const GenericLine<2>::vertex_type& n)const{
	// we use the formula from
	// https://brilliant.org/wiki/dot-product-distance-between-point-and-a-line/
	// to calculate the distance
	
	real_t A = -slope();
	real_t C = -factor();
	real_t B = 1.0;
	
	return std::fabs(A * n[0] + B * n[1] + C)/std::sqrt(bitrl::utils::maths::sqr(A) + bitrl::utils::maths::sqr(B));
	
}

}
}
}
#endif // GENERIC_LINE_H
