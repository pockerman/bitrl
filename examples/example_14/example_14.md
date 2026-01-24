\page bitrl_example_14 Example 14 Simulate a differential drive system with Chrono

In this example we will create a differential drive system and simulate it using the
<a href="https://github.com/projectchrono/chrono">Chrono</a> library.
Specifically, the robot we will simulate has

- Two motorised wheels
- An ultrasound sensor

In order to be able to run this example you need to configure bitrl with Chrono support. You will also
need the <a herf="https://irrlicht.sourceforge.io/">Irrlicht</a> library for visualising the robot.

The simulation that we will be developing is very simple since the purpose of this example is to
introduce some core components of the Chrono engine
