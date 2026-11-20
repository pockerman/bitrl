\page bitrl_example_monte_carlo_integration Monte Carlo integration

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

\f[
I=\int_a^b h(x) dx
\f]

If \f$f\f$ is a polynomial or a trigonometric function, then this integral can be calculated in closed form. 
However, in many cases there may not be  a closed for solution for \f$I\f$. Numerical techniques, such as <a href="https://en.wikipedia.org/wiki/Gaussian_quadrature">Gaussian quadrature</a> or the the <a href="https://en.wikipedia.org/wiki/Trapezoidal_rule">trapezoid rule</a> can  be 
employed in order to evaluate \f$I\f$. Monte Carlo integration is yet another techinque for evaluating complex integrals that is
notable for its simplicity and generality [1].

Let's begine by rewriting \f$I\f$ as follows

\f[
I=\int_a^b \omega(x)f(x) dx
\f]

where \f$\omega=h(x)(b-a)\f$ and \f$f(x) = 1/(b-a)\f$ i.e. \f$f\f$ is the probability density for a uniform random variable over \f$(a,b)\f$ [1]. 
Recall that the expectation for a continuous variable \f$X\f$ is given by

\f[E\left[X\right]=\int xf(x)dx\f]

Hence, 

\f[I=E\left[\omega(X)\right]\f]

This is the basic Monte Carlo integration method [1]. In order to evaluate the integral \f$I\f$, we evaluate the following expression

\f[\hat{I} = \frac{1}{n}\sum_{i=1}^{N}\omega(x_i)\f]

where \f$x \sim U(a,b)\f$. By the <a href="https://en.wikipedia.org/wiki/Law_of_large_numbers">law of large numbers</a> it follows, [1],

\f[
\hat{I}\rightarrow E\left[\omega(X)\right] = I
\f]

Notice that the law of large numbers provides us with probability convergence. 
Hence \f$\hat{I}\f$ will converge in probability to \f$I\f$. The standard error, \f$\hat{se}\f$, for the estimate is [1]

\f[
\hat{se} = \frac{s}{\sqrt{n}}
\f]

where

\f[
s^2  = \frac{\sum_{i}^{N}(\omega(x_i) - \hat{I} )^2}{n - 1}
\f]

A \f$1-\alpha\f$ confidence interval for the estimate is given from, [1], 

\f[
\hat{I} \pm z_{\alpha/2}\hat{se}
\f]

## The driver code

The driver code for this tutorial is shown below. 


\include monte_carlo_integration/monte_carlo_integration.cpp

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
