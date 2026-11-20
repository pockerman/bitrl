\page bitrl_example_7 bitrl Example 7: Importance sampling

The vanilla Monte Carlo method we saw in example <a href="../intro_example_1/intro_example_1.md">Monte Carlo intergration</a>
requires that we sample from a known distribution $f$. However, there may be cases  where it is difficult to sample from it.

In this example we will look at <a href="https://astrostatistics.psu.edu/su14/lectures/cisewski_is.pdf">imporatnce sampling</a> which 
allows us to overcome the dificculty of sampling from a difficult distribution.

## Importance sampling

Let us consider once again the integral 
$$I=\int_a^b h(x) dx$$

and rewrite it as 

$$I=\int_a^b \omega(x)f(x)$$

Importance sampling introduces a new probability distribution $g$, also known as the proposal distribution [2], 
that it is easier to  sample from. Thus we rewrite the integral as

$$I=\int_a^b \frac{\omega(x)f(x)}{g(x)}g(x)dx=E_g \left[Y \right]$$

where $Y$ is the random variable defined by

$$Y=\frac{\omega(x)f(x)}{g(x)}$$

We can now sample from $g$ and estimate $I$ as

$$\hat{I}=\frac{1}{N}\sum_i Y_i$$

Just like we did in the Monte Carlo integration section, we can use the law of 
large numbers and show that $\hat{I}\rightarrow I$ in probability.

In importance sampling we draw samples from $g$ and re-weight the integral using importance weights so
that the correct distribution is targeted [2]. However, $g$ in general has to have a similar shape with $f$. 
Moreover, it has to  have thicker  tails than $f$ otherwise the integral may become infinite [1]. 
Indeed, consider the second moment of $Y$:

$$E_g\left[ Y^2 \right]=\int Y^2g(x)dx=\int \frac{\omega^2(x)f^2(x)}{g(x)}dx $$

Thinner tails for $g$ means that it goes fatser to zero than what $f$ does. 

All in all, a good choice for $g$ is a distribution that is similar to $f$ but with thicker tails. In fact, the optimal choice for $g$ is given by the following theorem [1]

----
**Theorem**

The choice of $g$ that minimizes the variance of $\hat{I}$ is

$$g(x)=\frac{|h(x)|f(x)}{\int |h(s)|f(s)ds}$$

----


The example we will consider is taken from [1]. We want to estimate the 
following probability; $P(Z > 3)$ where $Z\sim N(0,1)$ This is just the integral:

$$P(Z > 3) = \int_{3}^{+\infty}f(x)dx = \int_{-\infty}^{+\infty}h(x)f(x)dx$$

where $h(x)$ is 1 if $x > 3$ and 0 otherwise and $f(x)$ is the PDF for the standard normal distribution.

## The driver code

The driver code for this tutorial is shown below.

```cpp
#include "cubeai/base/cubeai_types.h"
#include "cubeai/utils/iteration_counter.h"
#include "cubeai/maths/statistics/distributions/normal_dist.h"
#include "cubeai/maths/vector_math.h"
#include <boost/log/trivial.hpp>

#include <cmath>
#include <utility>
#include <tuple>
#include <iostream>
#include <random>
#include <algorithm>

namespace intro_example_7
{

using cubeai::real_t;
using cubeai::uint_t;
using cubeai::DynMat;
using cubeai::DynVec;
using cubeai::maths::stats::NormalDist;
using cubeai::utils::IterationCounter;

// we will sample from the normal distribution
// with mu = 4.0 and std = 1.0
const real_t MU = 4.0;
const real_t STD = 1.0;

// sample size we draw per iteration
const uint_t N = 100;

// how many iterations to run
const uint_t ITERATIONS = 1000;

const uint_t SEED = 42;


// simple function that computes the
// value of h at a given point
real_t h(real_t x){
    return 1.0 ? x > 3.0: 0.0;
}

}

int main() {

    using namespace intro_example_7;

    BOOST_LOG_TRIVIAL(info)<<"Starting example...";
	
	// simple object to control iterations
	IterationCounter counter(ITERATIONS);
	NormalDist dist(MU, STD);
	NormalDist proposal_dist(0.0, 1.0);
	
	std::vector<real_t> intergals;
	intergals.reserve(ITERATIONS);
	
	while(counter.continue_iterations()){
		
		real_t integral = 0.0;
		
		// sample from the distribution
		auto sample = dist.sample_many(N, SEED);
		
		// for every point in the sample compue
		// the PDF value
		for(auto p: sample){
			auto nom = h(p) * proposal_dist.pdf(p);
			auto denom = dist.pdf(p);
			auto val = nom / denom;
			integral += val;
		}
		
		intergals.push_back(integral / static_cast<real_t>(N));
		
	}
	
	auto E_I = cubeai::maths::mean(intergals.begin(), intergals.end(), true);
	auto V_I = cubeai::maths::variance(intergals.begin(), intergals.end(), true);
	BOOST_LOG_TRIVIAL(info)<<"E[I]="<<E_I;
	BOOST_LOG_TRIVIAL(info)<<"V[I]="<<V_I;
	BOOST_LOG_TRIVIAL(info)<<"Finished example...";
    return 0;
}



```


Running the driver code produces the following

```
[2024-12-07 11:21:45.653601] [0x00007fdf0c05d000] [info]    Starting example...
[2024-12-07 11:21:45.674922] [0x00007fdf0c05d000] [info]    E[I]=0.00146478
[2024-12-07 11:21:45.674941] [0x00007fdf0c05d000] [info]    V[I]=3.72444e-34
[2024-12-07 11:21:45.674954] [0x00007fdf0c05d000] [info]    Finished example...

```

## References

1. Larry Wasserman, _All of Statistics. A Concise Course in Statistical Inference_, Springer 2003.