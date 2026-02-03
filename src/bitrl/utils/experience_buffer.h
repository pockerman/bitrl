#ifndef EXPERIENCE_BUFFER_H
#define EXPERIENCE_BUFFER_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <boost/noncopyable.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <random>

namespace bitrl{
namespace utils {

/**
  * @brief The ExperienceBuffer class. A buffer based on
  * boost::circular_buffer to accumulate items of type ExperienceType.
  * see for example the A2C algorithm in A2C.h and rl_example_15
  */
template<typename ExperienceType>
class ExperienceBuffer: private boost::noncopyable{

public:
	
    static const uint_t DEFAULT_CAPACITY = 100;

    typedef ExperienceType value_type ;
    typedef ExperienceType experience_type;


    typedef typename boost::circular_buffer<ExperienceType>::iterator iterator;
    typedef typename boost::circular_buffer<ExperienceType>::const_iterator const_iterator;
    typedef typename boost::circular_buffer<ExperienceType>::reverse_iterator reverse_iterator;
    typedef typename boost::circular_buffer<ExperienceType>::const_reverse_iterator const_reverse_iterator;
	
	///
    /// \brief ExperienceBuffer
    ///
    ExperienceBuffer();

    ///
    /// \brief ExperienceBuffer
    ///
    explicit ExperienceBuffer(uint_t capacity);

    ///
    /// \brief append Add the experience item in the buffer
    ///
    void append(const experience_type& experience);

    ///
    /// \brief size
    /// \return
    ///
    uint_t size()const noexcept{return static_cast<uint_t>(buffer_.size());}

    ///
    /// \brief capacity
    /// \return
    ///
    uint_t capacity()const noexcept{return static_cast<uint_t>(buffer_.capacity());}

    ///
    /// \brief empty. Returns true if the buffer is empty
    /// \return
    ///
    bool empty()const noexcept{return buffer_.empty();}

    ///
    /// \brief clear
    ///
    void clear()noexcept{buffer_.clear();}

    ///
    /// \brief operator []
    /// \param idx
    /// \return
    ///
    value_type& operator[](uint_t idx){return buffer_[idx];}
	
	
	///
	/// \brief Resize the buffer
	///
	void resize(uint_t new_size, const experience_type& item = experience_type());

    ///
    /// \brief operator []
    /// \param idx
    /// \return
    ///
    const value_type& operator[](uint_t idx)const{return buffer_[idx];}

    ///
    /// \brief sample. Sample batch_size experiences from the
    /// buffer and transfer them in the BatchTp container.
    ///
    template<typename BatchTp>
    void sample(uint_t batch_size, BatchTp& batch, uint_t seed=42)const;
	
	///
	/// \brief Copy the contents of the buffer to the given container
	///
	template<typename ContainerType>
	void get(ContainerType& container)const;
	
	///
	/// \brief Copy the contents of the buffer to the given container
	///
	template<typename ContainerType>
	ContainerType get()const;

    iterator begin(){return buffer_.begin();}
    iterator end(){return buffer_.end();}

    const_iterator begin()const{return buffer_.begin();}
    const_iterator end()const{return buffer_.end();}

    reverse_iterator rbegin(){return buffer_.rbegin();}
    reverse_iterator rend(){return buffer_.rend();}

    const_reverse_iterator rbegin()const{return buffer_.rbegin();}
    const_reverse_iterator rend()const{return buffer_.rend();}

private:

   ///
   /// \brief buffer_
   ///
   boost::circular_buffer<ExperienceType> buffer_;

};

template<typename ExperienceTp>
ExperienceBuffer<ExperienceTp>::ExperienceBuffer()
    :
      buffer_(ExperienceBuffer<ExperienceTp>::DEFAULT_CAPACITY)
{}

template<typename ExperienceTp>
ExperienceBuffer<ExperienceTp>::ExperienceBuffer(uint_t max_size)
    :
      buffer_(max_size)
{}

template<typename ExperienceTp>
void
ExperienceBuffer<ExperienceTp>::append(const experience_type& experience){

    buffer_.push_back(experience);
}

template<typename ExperienceTp>
void
ExperienceBuffer<ExperienceTp>::resize(uint_t new_size, const experience_type& item){
	buffer_.resize(new_size, item);
}

template<typename ExperienceTp>
template<typename BatchTp>
void
ExperienceBuffer<ExperienceTp>::sample(uint_t batch_size, 
									   BatchTp& batch, uint_t seed)const{
			
#ifdef BITRL_DEBUG
	 assert(!empty() && "Cannot sample from an empty buffer");
#endif

	if(batch_size == 0){
#ifdef BITRL_DEBUG
	 assert(false && "Cannot sample with zero batch_size");
#endif	

	return;
		
	}
										
	// uniformly initialize weights 
	std::vector<real_t> weights(size(), 1.0/static_cast<real_t>(size()));
    std::discrete_distribution<uint_t> distribution(weights.begin(), weights.end());
	
	std::mt19937 generator(seed);
	for(uint_t c=0; c < batch_size; ++c){
		
		auto idx = distribution(generator);
		batch.push_back(buffer_[idx]);
	}
    
}

template<typename ExperienceType>
template<typename ContainerType>
void 
ExperienceBuffer<ExperienceType>::get(ContainerType& container)const{
	
	for(auto val: buffer_){
		container.push_back(val);
	}
}

template<typename ExperienceType>
template<typename ContainerType>
ContainerType
ExperienceBuffer<ExperienceType>::get()const{
	
	ContainerType container_;
	container_.reserve(size());
	for(auto val: buffer_){
		container_.push_back(val);
	}
	
	return container_;
}

}
}

#endif // EXPERIENCE_BUFFER_H
