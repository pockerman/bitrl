#ifndef RLENVS_TYPES_V2_H
#define RLENVS_TYPES_V2_H

/**
 * \file rlenvs_types_v2.h
 * 
 */

#include "bitrl/rlenvscpp_config.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <string>
#include <cstddef>
#include <vector>
#include <ostream>


namespace bitrl {

///
/// \brief real_t
///
typedef double real_t;

///
/// \brief float
///
typedef float float_t;

///
/// \brief integer type
///
typedef int int_t;

///
/// \brief long int type
///
typedef long int lint_t;

///
/// \brief uint_t
///
typedef std::size_t uint_t;

///
/// \brief Dynamically sized matrix to use 
/// around the library
///
template<typename T>
using DynMat = Eigen::MatrixX<T>;

///
/// \brief Square matrix with elements of type T
///
template<typename T, uint_t N>
using SquareMat = Eigen::Matrix<T, N, N>;

///
/// \brief General fixed size matrix
///
template<typename T, uint_t N, uint_t M>
using Mat = Eigen::Matrix<T, N, M>;

///
/// \brief Dynamic×3 matrix of type double. 
/// 
using RealMat3d = Eigen::Matrix3<real_t>;

///
/// \brief Dynamic×3 matrix of type float. 
///
using FloatMat3d = Eigen::Matrix3<float_t>;

///
/// \brief Dynamically sized row vector
///
template<typename T>
using DynVec = Eigen::RowVectorX<T>;

///
/// \brief single precision floating point vector 
///
using FloatVec = DynVec<float_t>;

///
/// \brief double precision floating point vector
///
using RealVec = DynVec<real_t>;

///
/// \brief single precision std::vector
///
using STD_FloatVec = std::vector<float_t>;

///
/// \brief double precision std::vector
///
using STD_RealVec = std::vector<real_t>;

///
/// \brief Column vector. Some maths operations
/// are easier using column vectors rather than DynVec
///
template<typename T>
using ColVec = Eigen::VectorX<T>;

///
/// \brief Dynamically sized column vector  
///
using RealColVec = ColVec<real_t>;

///
/// \brief Dynamically sized column vector  
///
using FoatColVec = ColVec<float_t>;

///
/// \brief 3D column vector
///
using RealColVec3d = Eigen::Vector3d;

///
/// \brief 3D column vectpr
///
using FloatColVec3d = Eigen::Vector3f;


///
/// \struct IntegralRange 
/// \brief A range of integer values in
/// [s, e]
///
template<uint_t s, uint_t e>
struct IntegralRange
{
	typedef uint_t value_type;
	static constexpr uint_t S = s;
	static constexpr uint_t E = e;
	static constexpr uint_t size = e - s;
};

///
/// \struct RealRange 
/// \brief A range of double precision floating point values
///
template<real_t s, real_t e>
struct RealRange
{
	typedef real_t value_type;
	static constexpr real_t S = s;
	static constexpr real_t E = e;
	static constexpr real_t size = e - s;
};


///
/// \struct Null
/// \brief Null placeholder
///
struct Null{};

/// \enum DeviceType
/// \brief Enumeration of various device types.
///
enum class DeviceType {INVALID_TYPE=0, CPU=1, GPU=2} ;

inline
std::ostream& operator<<(std::ostream& out,
                         const Null&){

	out<<"null";
	return out;
}

}

#endif // TYPES_H

