
\page bitrl_example_1 bitrl Example 1: Monte Carlo integration

In this first example we explore some of the basic elements of _bitrl_. In particular,
we will see how to read a json file, how to use boost logging, various types exposed by the library.
We will do so by using a simple <a href="https://en.wikipedia.org/wiki/Monte_Carlo_integration">Monte Carlo integration</a> example.


## Monte Carlo intergration

Frequently in applications we need to evaluate integrals for which no analytical solution exists. 
Numerical methods can help us overcome this. Monte Carlo is just one of these methods. 
The method works due to the <a href="https://en.wikipedia.org/wiki/Law_of_large_numbers">law of large numbers</a>. 
Compared to a standard numerical method, the method may not be especially efficient in one dimension, 
but it becomes increasingly efficient as the dimensionality of the integral grows.

Let's assume that we want to evaluate the integral

$$
I=\int_a^b h(x) dx
$$

If $f$ is a polynomial or a trigonometric function, then this integral can be calculated in closed form. 
However, in many cases there may not be  a closed for solution for $I$. Numerical techniques, such as <a href="https://en.wikipedia.org/wiki/Gaussian_quadrature">Gaussian quadrature</a> or the the <a href="https://en.wikipedia.org/wiki/Trapezoidal_rule">trapezoid rule</a> can  be 
employed in order to evaluate $I$. Monte Carlo integration is yet another techinque for evaluating complex integrals that is
notable for its simplicity and generality [1].

Let's begine by rewriting $I$ as follows

$$
I=\int_a^b \omega(x)f(x) dx
$$

where $\omega=h(x)(b-a)$ and $f(x) = 1/(b-a)$ i.e. $f$ is the probability density for a uniform random variable over $(a,b)$ [1]. 
Recall that the expectation for a continuous variable $X$ is given by

$$E\left[X\right]=\int xf(x)dx$$

Hence, 

$$I=E\left[\omega(X)\right]$$

This is the basic Monte Carlo integration method [1]. In order to evaluate the integral $I$, we evaluate the following expression

$$\hat{I} = \frac{1}{n}\sum_{i=1}^{N}\omega(x_i)$$

where $x \sim U(a,b)$. By the <a href="https://en.wikipedia.org/wiki/Law_of_large_numbers">law of large numbers</a> it follows, [1],

$$
\hat{I}\rightarrow E\left[\omega(X)\right] = I
$$

Notice that the law of large numbers provides us with probability convergence. 
Hence $\hat{I}$ will converge in probability to $I$. The standard error, $\hat{se}$, for the estimate is [1]

$$
\hat{se} = \frac{s}{\sqrt{n}}
$$

where

$$
s^2  = \frac{\sum_{i}^{N}(\omega(x_i) - \hat{I} )^2}{n - 1}
$$

A $1-\alpha$ confidence interval for the estimate is given from, [1], 

$$
\hat{I} \pm z_{\alpha/2}\hat{se}
$$

## The driver code

The driver code for this tutorial is shown below. 


@code
#include "bitrl/base/cubeai_types.h"
#include "bitrl/utils/iteration_counter.h"
#include "bitrl/geom_primitives/shapes/circle.h"
#include "bitsim/extern/nlohmann/json/json.hpp"

#include <boost/log/trivial.hpp>
#include <iostream>
#include <random>
#include <fstream>

namespace intro_example_1
{

using cubeai::real_t;
using cubeai::uint_t;
using cubeai::utils::IterationCounter;
using cubeai::geom_primitives::Circle;

using json = nlohmann::json;

const std::string CONFIG = "config.json";

// read the JSON file
json
load_config(const std::string& filename){

  std::ifstream f(filename);
  json data = json::parse(f);
  return data;
}


}

int main() {

    using namespace intro_example_1;

    try{

        BOOST_LOG_TRIVIAL(info)<<"Reading configuration file...";

        // load the json configuration
        auto data = load_config(CONFIG);

        // read properties from the configuration
        const auto R = data["R"].template get<real_t>();
        const auto N_POINTS = data["N_POINTS"].template get<uint_t>();
        const auto SEED = data["SEED"].template get<uint_t>();
        const auto X = data["X"].template get<real_t>();
        const auto Y = data["Y"].template get<real_t>();

        // create a circle
        Circle c(R, {X, Y});

        // simple object to control iterations
        IterationCounter counter(N_POINTS);

        // how many points we found in the Circle
        auto points_inside_circle = 0;

        // the box has side 2
        const real_t SQUARE_SIDE = R*2.0;
        std::uniform_real_distribution dist(0.0,SQUARE_SIDE);
        std::mt19937 gen(SEED);

        BOOST_LOG_TRIVIAL(info)<<"Starting computation...";
        while(counter.continue_iterations()){
            auto x = dist(gen);
            auto y = dist(gen);
            if(c.is_inside(x,y, 1.0e-4)){
              points_inside_circle += 1;
            }
        }

        BOOST_LOG_TRIVIAL(info)<<"Finished computation...";
        auto area = (static_cast<real_t>(points_inside_circle) / static_cast<real_t>(N_POINTS)) * std::pow(SQUARE_SIDE, 2);
        BOOST_LOG_TRIVIAL(info)<<"Circle area calculated with:" <<N_POINTS<<" is: "<<area;
        BOOST_LOG_TRIVIAL(info)<<"Circle area: "<<c.area();
    }
    catch(std::exception& e){
        BOOST_LOG_TRIVIAL(error)<<e.what();
    }
    catch(...){
        BOOST_LOG_TRIVIAL(error)<<"Unknown exception occured";
    }

   return 0;
} 
@endcode

Running the code above produces the following output

@code
[2024-05-30 17:51:47.253901] [0x00007fe2bbb7b540] [info]    Reading configuration file...
[2024-05-30 17:51:47.254017] [0x00007fe2bbb7b540] [info]    Starting computation...
[2024-05-30 17:51:47.258199] [0x00007fe2bbb7b540] [info]    Finished computation...
[2024-05-30 17:51:47.258222] [0x00007fe2bbb7b540] [info]    Circle area calculated with:100000 is: 3.14716
[2024-05-30 17:51:47.258226] [0x00007fe2bbb7b540] [info]    Circle area: 3.14159
@endcode


## References

1. Larry Wasserman, _All of Statistics. A Concise Course in Statistical Inference_, Springer 2003.
