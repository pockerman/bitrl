#ifndef SPACE_TYPE_H
#define SPACE_TYPE_H

#include "bitrl/bitrl_types.h"
#include <random>
#include <vector>

#include <type_traits>

namespace bitrl {
namespace envs{
	
///
/// \brief A scalar discrete space can be used
/// to denote a space that only has a single value
/// at each time. For example consider the FrozenLake environment
/// then both the state and the action spaces are ScalarDiscreteSpace
///
template<uint_t s, uint_t e>	
struct ScalarDiscreteSpace{
	
	///
    /// \brief item_t
    ///
    typedef uint_t space_item_type;
	
	///
	/// \brief The limits of the space
	///
	static constexpr IntegralRange<s, e> limits = IntegralRange<s, e>();
	
    ///
    /// \brief The overall size of the space meaning
	/// how many elements can potentially the space have
    ///
    static constexpr uint_t size = IntegralRange<s, e>::size;
	
	///
    /// \brief sample
    /// \return
    ///
    static space_item_type sample(bool use_end);

    ///
    /// \brief sample
    /// \param seed
    /// \return
    ///
    static space_item_type sample(uint_t seed, bool use_end);
	
	///
    /// \brief sample
    /// \param seed
    /// \return
    ///
    static std::vector<space_item_type> sample(uint_t seed, uint_t size, bool use_end);
	
};

template<uint_t s, uint_t e>
typename ScalarDiscreteSpace<s, e>::space_item_type
ScalarDiscreteSpace<s, e>::sample(bool use_end){

	auto E = IntegralRange<s, e>::E;
	
	if(!use_end){
		E -= 1;
	}
	
    std::uniform_int_distribution<> dist(IntegralRange<s, e>::S, E);
    std::random_device rd;
    std::mt19937 gen(rd());
    return dist(gen);
}

template<uint_t s, uint_t e>
typename ScalarDiscreteSpace<s, e>::space_item_type
ScalarDiscreteSpace<s, e>::sample(uint_t seed, bool use_end){
	
	auto E = IntegralRange<s, e>::E;
	
	if(!use_end){
		E -= 1;
	}

    std::uniform_int_distribution<> dist(IntegralRange<s, e>::S, E);
    std::mt19937 gen(seed);
    return dist(gen);
}

template<uint_t s, uint_t e>
std::vector<typename ScalarDiscreteSpace<s, e>::space_item_type>
ScalarDiscreteSpace<s, e>::sample(uint_t seed, uint_t size, bool use_end){
	
	auto E = IntegralRange<s, e>::E;
	
	if(!use_end){
		E -= 1;
	}

	std::vector<typename ScalarDiscreteSpace<s, e>::space_item_type> vals_;
	vals_.reserve(size);
	
	std::uniform_int_distribution<> dist(IntegralRange<s, e>::S, E);
    std::mt19937 gen(seed);
	for(uint_t i=0; i<size; ++i){
		vals_.push_back(dist(gen));
	}
	
    return vals_;
}



template<uint_t Size>
struct ContinuousScalareSpace
{
	///
    /// \brief item_t
    ///
    typedef real_t space_item_type;
	
	///
    /// \brief The overall size of the space meaning
	/// how many elements can potentially the space have
    ///
    static constexpr uint_t size = Size;
};

///
/// \brief A space that represents a range of discrete 
///
template<real_t S, real_t E>
struct BoundedContinuousScalarSpace
{
	///
    /// \brief item_t
    ///
    typedef real_t space_item_type;
	
	///
    /// \brief The overall size of the space meaning
	/// how many elements can potentially the space have
    ///
    static constexpr uint_t size = 1;
	
	///
	/// \brief The boundaries the scalar value can assume
	///
	static constexpr RealRange<S, E> limits = RealRange<S, E>();
};


template<uint_t Size, typename T=real_t>
struct ContinuousVectorSpace
{
	static_assert(std::is_floating_point_v<T> == true && "Floating point type is expected");
	
	///
    /// \brief The overall size of the space meaning
	/// how many elements can potentially the space have
    ///
    static constexpr uint_t size = Size;
	
	///
    /// \brief item_t
    ///
    typedef std::vector<T> space_item_type;
};





/*template<uint_t SpaceSize>
typename DiscreteSpace<SpaceSize>::space_item_type
DiscreteSpace<SpaceSize>::sample(uint_t seed){

    std::uniform_int_distribution<> dist(0, SpaceSize - 1);
    std::mt19937 gen(seed);
    return dist(gen);
}

template<uint_t SpaceSize>
std::vector<typename DiscreteSpace<SpaceSize>::space_item_type>
DiscreteSpace<SpaceSize>::sample(uint_t seed, uint_t size){

	std::vector<typename DiscreteSpace<SpaceSize>::space_item_type> vals_;
	vals_.reserve(size);
	
	std::uniform_int_distribution<> dist(0, SpaceSize - 1);
    std::mt19937 gen(seed);
	for(uint_t i=0; i<size; ++i){
		
		vals_.push_back(dist(gen));
	}
	
    return vals_;
}*/

/*
template<typename T=real_t>	
struct ScalarContinuousSpace
{
	static_assert(std::is_floating_point_v<T> == true && "Floating point type is expected");
	
	///
    /// \brief item_t
    ///
    typedef T space_item_type;


    ///
    /// \brief size
    ///
    static constexpr uint_t size = 1;
	
};

///
/// \brief Discrete vector space all elements of the 
/// space should be of integral type. A discrete vector
/// can be used to specify a discrete state that is vector
/// valued see e.g. the Connect2 environment 
///
template<typename uint_t SpaceSize, typename VectorType>
struct DiscreteVectorSpace;

template<uint_t SpaceSize, typename T=uint_t>
struct DiscreteVectorSpace<SpaceSize, DynVec<T> >
{
	static_assert(std::is_integral_v<T> == true && "Integral type is expected");

    ///
    /// \brief item_t
    ///
    typedef std::vector<T> space_item_type;


    ///
    /// \brief size
    ///
    static constexpr uint_t size = SpaceSize;

    ///
    /// \brief sample
    /// \return
    ///
    static space_item_type sample();

    ///
    /// \brief sample
    /// \param seed
    /// \return
    ///
    static space_item_type sample(uint_t seed);
	
	///
    /// \brief sample
    /// \param seed
    /// \return
    ///
    static std::vector<space_item_type> sample(uint_t seed, uint_t size);
};


template<uint_t SpaceSize, typename T=uint_t>
struct DiscreteVectorSpace<SpaceSize, std::vector<T> >
{
	
	static_assert(std::is_integral_v<T> == true && "Integral type is expected");

    ///
    /// \brief item_t
    ///
    typedef std::vector<T> space_item_type;


    ///
    /// \brief size
    ///
    static constexpr uint_t size = SpaceSize;

    ///
    /// \brief sample
    /// \return
    ///
    static space_item_type sample();

    ///
    /// \brief sample
    /// \param seed
    /// \return
    ///
    static space_item_type sample(uint_t seed);
	
	///
    /// \brief sample
    /// \param seed
    /// \return
    ///
    static std::vector<space_item_type> sample(uint_t seed, uint_t size);
};

template<uint_t SpaceSize>
typename DiscreteSpace<SpaceSize>::space_item_type
DiscreteSpace<SpaceSize>::sample(){

    std::uniform_int_distribution<> dist(0, SpaceSize - 1);
    std::random_device rd;
    std::mt19937 gen(rd());
    return dist(gen);
}

template<uint_t SpaceSize>
typename DiscreteSpace<SpaceSize>::space_item_type
DiscreteSpace<SpaceSize>::sample(uint_t seed){

    std::uniform_int_distribution<> dist(0, SpaceSize - 1);
    std::mt19937 gen(seed);
    return dist(gen);
}

template<uint_t SpaceSize>
std::vector<typename DiscreteSpace<SpaceSize>::space_item_type>
DiscreteSpace<SpaceSize>::sample(uint_t seed, uint_t size){

	std::vector<typename DiscreteSpace<SpaceSize>::space_item_type> vals_;
	vals_.reserve(size);
	
	std::uniform_int_distribution<> dist(0, SpaceSize - 1);
    std::mt19937 gen(seed);
	for(uint_t i=0; i<size; ++i){
		
		vals_.push_back(dist(gen));
	}
	
    return vals_;
}

template<uint_t SpaceSize, typename ItemType = std::vector<real_t>>
struct ContinuousSpace{
	
	///
    /// \brief item_t
    ///
    typedef std::vector<real_t> space_item_type;

    ///
    /// \brief size. The number of members in the space
    ///
    static constexpr uint_t size = SpaceSize;
};

///
/// \brief The ContinuousSpace class.
///
//template<uint_t SpaceSize>
//struct ContinuousSpace<SpaceSize, std::vector<real_t> >
//{
//    ///
//    /// \brief item_t
//    ///
//    typedef std::vector<real_t> space_item_type;
//
//    ///
//    /// \brief size. The number of members in the space
//    ///
//    static constexpr uint_t size = SpaceSize;
//
//};



template<>
struct ContinuousSpace<1, Null>
{
	
	///
    /// \brief item_t
    ///
    typedef real_t space_item_type;

    ///
    /// \brief size. The number of members in the space
    ///
    static constexpr uint_t size = 1;

};
*/

template<uint_t SpaceSize, typename T=uint_t>
struct DiscreteVectorSpace
{
	static_assert(std::is_integral_v<T> == true && "Integral type is expected");
	
	///
    /// \brief item_t
    ///
    typedef std::vector<T> space_item_type;
	
	
	///
    /// \brief size. The number of members in the space
    ///
    static constexpr uint_t size = SpaceSize;
};


}
}
#endif