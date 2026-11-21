\page bitrl_example_importance_sampling Importance sampling

The vanilla Monte Carlo method we saw in example <a href="../intro_example_1/intro_example_1.md">Monte Carlo intergration</a>
requires that we sample from a known distribution \f$f\f$. However, there may be cases  where it is difficult to sample from it.

In this example we will look at <a href="https://astrostatistics.psu.edu/su14/lectures/cisewski_is.pdf">imporatnce sampling</a> which 
allows us to overcome the dificculty of sampling from a difficult distribution.

## Importance sampling

Let us consider once again the integral 
\f[I=\int_a^b h(x) dx\f]

and rewrite it as 

\f[I=\int_a^b \omega(x)f(x)\f]

Importance sampling introduces a new probability distribution \f$g\f$, also known as the proposal distribution [2], 
that it is easier to  sample from. Thus we rewrite the integral as

\f[I=\int_a^b \frac{\omega(x)f(x)}{g(x)}g(x)dx=E_g \left[Y \right]\f]

where \f$Y\f$ is the random variable defined by

\f[Y=\frac{\omega(x)f(x)}{g(x)}\f]

We can now sample from \f$g\f$ and estimate \f$I\f$ as

\f[\hat{I}=\frac{1}{N}\sum_i Y_i\f]

Just like we did in the Monte Carlo integration section, we can use the law of 
large numbers and show that \f$\hat{I}\rightarrow I\f$ in probability.

In importance sampling we draw samples from \f$g\f$ and re-weight the integral using importance weights so
that the correct distribution is targeted [2]. However, \f$g\f$ in general has to have a similar shape with \f$f\f$. 
Moreover, it has to  have thicker  tails than \f$f\f$ otherwise the integral may become infinite [1]. 
Indeed, consider the second moment of \f$Y\f$:

\f[E_g\left[ Y^2 \right]=\int Y^2g(x)dx=\int \frac{\omega^2(x)f^2(x)}{g(x)}dx \f]

Thinner tails for \f$g\f$ means that it goes fatser to zero than what \f$f\f$ does. 

All in all, a good choice for \f$g\f$ is a distribution that is similar to \f$f\f$ but with thicker tails. In fact, the optimal choice for \f$g\f$ is given by the following theorem [1]

----
**Theorem**

The choice of \f$g\f$ that minimizes the variance of \f$\hat{I}\f$ is

\f[g(x)=\frac{|h(x)|f(x)}{\int |h(s)|f(s)ds}\f]

----


The example we will consider is taken from [1]. We want to estimate the 
following probability; \f$P(Z > 3)\f$ where \f$Z\sim N(0,1)\f$ This is just the integral:

\f[P(Z > 3) = \int_{3}^{+\infty}f(x)dx = \int_{-\infty}^{+\infty}h(x)f(x)dx\f]

where \f$h(x)\f$ is 1 if \f$x > 3\f$ and 0 otherwise and \f$f(x)\f$ is the PDF for the standard normal distribution.

## The driver code

The driver code for this tutorial is shown below.

\include importance_sampling/importance_sampling.cpp


Running the driver code produces the following

```
[2024-12-07 11:21:45.653601] [0x00007fdf0c05d000] [info]    Starting example...
[2024-12-07 11:21:45.674922] [0x00007fdf0c05d000] [info]    E[I]=0.00146478
[2024-12-07 11:21:45.674941] [0x00007fdf0c05d000] [info]    V[I]=3.72444e-34
[2024-12-07 11:21:45.674954] [0x00007fdf0c05d000] [info]    Finished example...

```

## References

1. Larry Wasserman, _All of Statistics. A Concise Course in Statistical Inference_, Springer 2003.