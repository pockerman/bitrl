#ifndef BERNOULLI_DIST_H
#define BERNOULLI_DIST_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"

#include <vector>
#include <random>

namespace bitrl {
namespace utils{
namespace maths {
namespace stats {

///
/// \brief class BernoulliDist. Wrapper to std::bernoulli_distribution
///	
class BernoulliDist
{
public:
		
	///	
	/// \breif The return type every time we call pdf, sample
	///
	typedef bool result_type;  
	
	///
	/// \brief Constructor
	///
	explicit BernoulliDist(real_t p=0.5);
		
	///
	/// \brief Sample from the distribution
	///
	result_type sample() const;
	
	///
	/// \brief Sample from the distribution
	///
	result_type sample(uint_t seed) const;
	
	///
	/// \brief sample from the distribution
	///
	std::vector<result_type> sample_many(uint_t size) const;
	
	///
	/// \brief sample from the distribution
	///
	std::vector<result_type> sample_many(uint_t size, uint_t seed) const;
	
	///
	/// \brief The mean value of the distribution
	///
	real_t mean()const{return p_;}
	
	///
	/// \brief The STD of the distribution
	///
	real_t variance()const{return p_*(1.0 - p_);}
	
	///
	/// \brief Reset the underlying distribution
	///
	void reset(){dist_.reset();}
	
	///
	/// \brief Reset the distribution with a new p
	///
	void reset(real_t p);
	
private:
	
	real_t p_;
	mutable std::bernoulli_distribution dist_;
	
};
	
}
}
}
}

#endif