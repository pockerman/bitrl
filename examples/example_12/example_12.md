\page bitrl_example_12 BitRL Example 12 A rigid body simulation with Chrono

Example \ref bitrl_example_11 discussed _ChBody_. Specifically, how to create a _ChBodyEasyBox_.
Understanding rigid bodies is fundamental to robotics and further examples in this series will dive deeper into
this subject. In this example we want to create a simple simulation of a <a href="https://en.wikipedia.org/wiki/Differential_wheeled_robot">differential drive robot</a>.
According to wikipedia:

_A differential wheeled robot is a mobile robot whose movement is based on two separately 
driven wheels placed on either side of the robot body. It can thus change its direction by varying the 
relative rate of rotation of its wheels and hence does not require an additional steering motion. 
Robots with such a drive typically have one or more caster wheels to prevent the vehicle from tilting._

Thus we will build a robot with two motorised wheels and one passive caster wheel useful for balancing the robot.
