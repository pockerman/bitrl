\page bitrl_example_6 bitrl Example 6: Toy Markov chain

This example uses to toy Markov chain model to introduce some operations on the
main linear algebra backend that _bitrl_ is using namely <a href="https://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a>.

## The driver code

The driver code for this tutorial is shown below.

\include example_6/example_6.cpp

Running the driver code above produces the following output

@code
After 3 steps...
[0.844 , 0.156]
[0.78 , 0.22]
After 50 steps...
[0.833333 , 0.166667]
[0.833333 , 0.166667]
After 100 steps...
[0.833333 , 0.166667]
[0.833333 , 0.166667]
v_3=0.844 0.156
    0     0
v_50=0.833333 0.166667
       0        0
v_100=0.833333 0.166667
       0        0
v_3=0.422 0.078
0.422 0.078
v_50= 0.416667 0.0833333
 0.416667 0.0833333
v_100= 0.416667 0.0833333
 0.416667 0.0833333
@endcode

