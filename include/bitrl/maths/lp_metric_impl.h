#ifndef LP_METRIC_IMPL_H
#define LP_METRIC_IMPL_H

#include <cmath>

namespace bitrl{
namespace maths {

template<int P, bool TTakeRoot>
real_t LpMetric<P,TTakeRoot>::tolerance_value = 1.0e-8;

//the general implementation
template<int P, bool TTakeRoot>
real_t
LpMetric<P,TTakeRoot>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){

  real_t sum = 0.0;
  for (uint_t i = 0; i < v1.size(); i++){
      sum += std::pow(std::fabs(v1[i] - v2[i]), P);
  }

  if (!TTakeRoot) // The compiler should optimize this correctly at compile-time.
     return sum;

  return std::pow(sum, (1.0 / P));
}

//the general implementation
template<int P, bool TTakeRoot>
real_t
LpMetric<P,TTakeRoot>::evaluate(const std::vector<real_t>& v1, const std::vector<real_t>& v2){

  real_t sum = 0.0;
  for (uint_t i = 0; i < v1.size(); i++){
      sum += std::pow(std::fabs(v1[i] - v2[i]), P);
  }

  if (!TTakeRoot) // The compiler should optimize this correctly at compile-time.
     return sum;

  return std::pow(sum, (1.0 / P));
}

// L1-metric specializations; the root doesn't matter.
template<>
real_t
LpMetric<1, true>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){
  return blaze::l1Norm(v1 - v2);
}

template<>
real_t
LpMetric<1, false>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){
  return blaze::l1Norm(v1 - v2);
}

// L2-metric specializations.
template<>
real_t
LpMetric<2, true>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){
  return std::sqrt(blaze::sqrNorm(v1 - v2));
}

template<>
real_t
LpMetric<2, false>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){
  return blaze::sqrNorm(v1 - v2);
}

// L3-metric specialization (not very likely to be used, but just in case).
template<>
real_t
LpMetric<3, true>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){
  return std::pow(blaze::reduce(blaze::pow(blaze::abs(v1 - v2), 3.0), blaze::Add()), 1.0 / 3.0);
}

template<>
real_t
LpMetric<3, false>::evaluate(const DynVec<real_t>& v1, const DynVec<real_t>& v2){
  return std::pow(blaze::reduce(blaze::pow(blaze::abs(v1 - v2), 3.0), blaze::Add()), 1.0 / 3.0);
}


template<int P, bool TTakeRoot>
real_t
LpMetric<P, TTakeRoot>::operator()(const DynVec<real_t>& v1, const DynVec<real_t>& v2)const{
  return LpMetric<P, TTakeRoot>::evaluate(v1,v2);
}

template<>
real_t
LpMetric<1, true>::evaluate(const std::vector<real_t>& v1, const std::vector<real_t>& v2){

    real_t sum = 0.0;

    for(uint_t i=0; i<v1.size(); ++i){
        sum += std::fabs(v1[i] - v2[i]);
    }

    return sum;

}

template<>
real_t
LpMetric<1, false>::evaluate(const std::vector<real_t>& v1, const std::vector<real_t>& v2){

    return LpMetric<1, true>::evaluate(v1 , v2);
}

template<>
real_t
LpMetric<2, true>::evaluate(const std::vector<real_t>& v1, const std::vector<real_t>& v2){

    real_t sum = 0.0;

    for(uint_t i=0; i<v1.size(); ++i){
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return std::sqrt(sum);

}

template<>
real_t
LpMetric<2, false>::evaluate(const std::vector<real_t>& v1, const std::vector<real_t>& v2){

    real_t sum = 0.0;

    for(uint_t i=0; i<v1.size(); ++i){
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sum;
}

}
}
#endif // LP_METRIC_IMPL_H
