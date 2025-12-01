
``rlenvscpp``
===============

``rlenvscpp`` is an effort to provide implementations and wrappers of environments suitable for training reinforcement learning agents
using  C++. In addition, the library provides various utilities such as experiment tracking,
representing trajectories  via waypoints and simple implementation of popular dynamics such as 
quadrotor dynamics.

Various RL algorithms using the environments can be found at `cuberl <https://github.com/pockerman/cuberl/tree/master>`_.

Gymnasium environments
-----------------------

Currently, ``rlenvscpp`` provides the following Gymnasium environments.
Note that you will need to have Gymnasium installed.

* FrozenLake 4x4 map: `example_1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_1/example_1.cpp>`_
* FrozenLake 8x8 map: `example_1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_1/example_1.cpp>`_
* Blackjack: `example_1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_1/example_1.cpp>`_
* CliffWalking: `example_1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_1/example_1.cpp>`_
* CartPole: TODO
* MountainCar: TODO
* Taxi: `example_1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_1/example_1.cpp>`_ 
* Pendulum: `example_1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_6/example_6.cpp>`_
* LunarLander: `box2d <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_6/example_6.cpp>`_
* Acrobot: TODO
* LunarLander: `https://github.com/pockerman/rlenvscpp/blob/master/examples/box2d/box2d_example.cpp`_

The Gymnasium (former OpenAI-Gym) environments utilise a REST API to communicate requests to/from the 
environment and ``rlenvscpp``.

Gymnasium vector environments
-----------------------------

Some environments have a vector implementation meaning multiple instances of the same
environment. Currently, ``rlenvscpp`` provides the following vector environments: 

* AcrobotV:  `example_8 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_8/example_8.cpp>`_ 

Webots environments
--------------------------

In order to use the Webots environments, you need to have `webots <https://cyberbotics.com/#cyberbotics>`_
installed on your machine. You also need to set the variable ``ENABLE_WEBOTS`` to ``ON`` when configuring the 
library.

* EpuckSimpleGridWorld:  `world 1 <https://github.com/pockerman/rlenvscpp/blob/master/examples/webots/world_1/controllers/e_puck_controller/e_puck_controller.cpp>`_ 

Working with webots-based environments is a bit different. See the dedicated instructions.

Miscellaneous environments
--------------------------

* GymWalk: TODO             
* gym-pybullet-drones: TODO
* GridWorld: `example_5 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_5/example_5.cpp>`_
* Connect2:  `example_7 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_7/example_7.cpp>`_ 
* MultiArmedBandits:   TODO 

Dynamics 
---------

Apart from the exposed environments, ``rlenvscpp`` exposes classes that 
describe the dynamics of some popular rigid bodies:

* Differential drive:  `example_9 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_9/example_9.cpp>`_
* Quadrotor:           `example_10 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_10/example_10.cpp>`_
* Bicycle vehicle:  TODO


Miscellaneous examples
----------------------

* Environment trajectory: `example_3 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_3/example_3.cpp>`_  
* WaypointTrajectory    : `example_11 <https://github.com/pockerman/rlenvscpp/blob/master/examples/example_11/example_11.cpp>`_
* TensorboardServer     : `example 12 <https://github.com/pockerman/rlenvscpp/tree/master/examples/example_12>`_

