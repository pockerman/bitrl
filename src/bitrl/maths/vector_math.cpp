#include "bitrl/maths/vector_math.h"
#include <cmath>
#include <algorithm>

namespace bitrl{
namespace maths{

std::vector<real_t>
logspace(real_t start, real_t end, uint_t num, real_t base){

    std::vector<real_t> logspace(num, 0.0);
    logspace[0] = std::pow(base, start);
    logspace[num - 1 ] = std::pow(base, end);
    real_t dx = (end - start) / static_cast<real_t>(num - 1);

    for (int i = 1; i < num - 1; ++i){
        auto point = start + i*dx;
        logspace[i] = std::pow(base, point);
    }

    return logspace;

}

std::vector<real_t>
standardize(const std::vector<real_t>& vals, real_t tol){
	
	auto mean_val = mean(vals);
	auto var = variance(vals.begin(), vals.end()) + tol;
	
	std::vector<real_t> std_vals;
	std_vals.reserve(vals.size());
	
	auto standardize_ = [&std_vals, mean_val, var](const real_t& val){
		
		std_vals.push_back((val - mean_val) / var);
	};
	
	std::for_each(vals.begin(),
	              vals.end(),
				  standardize_);

	return std_vals;
	
}

}
}
