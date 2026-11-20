\page bitrl_example_chrono_smc_system Using chrono::ChSystemSMC

Example \ref bitsim_example_chrono_rigid_body_simulation showed you how to create a rigid body simulation in Chrono. We will extend this example
and show how to create a reinforcement learning environment that can be queried by reinforcement learning agents.

Reinforcement learning environments in _bitrl_ should implement the \ref bitrl::envs::EnvBase "`bitrl::envs::EnvBase`" class.
This is an abstract base class that defines the main endpoints that a reinforcement learning environment should implement.
We will keep things simple in this example and assume that the action space is discrete with three actions available.
We won't be doing anything meaningful in this example other than showing how a reinforcement learning environment can be 
implemented using Chrono. 

The state space is equally simple and returns the position of the robot. 
Below is the driver implementing this. To  a large extent this is the same as example \ref bitsim_example_chrono_rigid_body_simulation.
We have also introduced the headers _robot.h_ and _robot_env.h_ to separate concerns.

\include chrono_smc_system/chrono_smc_system.cpp

Running the code above produces the following: 

@code
[2026-03-15 17:49:19.445734] [0x00007fbe766b0800] [info]    At time: 0.002 position: 0.000830566  1.24587e-06  0.160031
[2026-03-15 17:49:19.462184] [0x00007fbe766b0800] [info]    At time: 0.004 position: 0.00166125  2.2391e-06  0.160032
[2026-03-15 17:49:19.472812] [0x00007fbe766b0800] [info]    At time: 0.006 position: 0.00249691  2.28551e-06  0.160009
[2026-03-15 17:49:19.490501] [0x00007fbe766b0800] [info]    At time: 0.008 position: 0.00333187  4.48252e-06  0.159995
[2026-03-15 17:49:19.505314] [0x00007fbe766b0800] [info]    At time: 0.01 position: 0.00416369  6.52889e-06  0.159995
[2026-03-15 17:49:19.522646] [0x00007fbe766b0800] [info]    At time: 0.012 position: 0.00499371  5.18348e-06  0.159992
[2026-03-15 17:49:19.538717] [0x00007fbe766b0800] [info]    At time: 0.014 position: 0.00582212  4.4504e-06  0.159994
[2026-03-15 17:49:19.555803] [0x00007fbe766b0800] [info]    At time: 0.016 position: 0.00664943  5.01632e-06  0.159993
[2026-03-15 17:49:19.573243] [0x00007fbe766b0800] [info]    At time: 0.018 position: 0.00747616  5.43181e-06  0.159994
[2026-03-15 17:49:19.589256] [0x00007fbe766b0800] [info]    At time: 0.02 position: 0.00830281  1.09076e-05  0.159984
[2026-03-15 17:49:19.605600] [0x00007fbe766b0800] [info]    At time: 0.022 position: 0.00912996  1.29684e-05  0.159984
[2026-03-15 17:49:19.623059] [0x00007fbe766b0800] [info]    At time: 0.024 position: 0.00995797  9.89451e-06  0.159999
[2026-03-15 17:49:19.639142] [0x00007fbe766b0800] [info]    At time: 0.026 position: 0.0107875  7.19161e-06  0.159997
[2026-03-15 17:49:19.657061] [0x00007fbe766b0800] [info]    At time: 0.028 position: 0.0116187  9.68353e-06  0.159997
[2026-03-15 17:49:19.673839] [0x00007fbe766b0800] [info]    At time: 0.03 position: 0.0124514  1.10808e-05  0.159997
[2026-03-15 17:49:19.690629] [0x00007fbe766b0800] [info]    At time: 0.032 position: 0.0132852  1.08047e-05  0.160001
[2026-03-15 17:49:19.706856] [0x00007fbe766b0800] [info]    At time: 0.034 position: 0.01412  1.23457e-05  0.159996
[2026-03-15 17:49:19.723398] [0x00007fbe766b0800] [info]    At time: 0.036 position: 0.0149556  1.28846e-05  0.159996
[2026-03-15 17:49:19.741549] [0x00007fbe766b0800] [info]    At time: 0.038 position: 0.0157916  1.17644e-05  0.160001
[2026-03-15 17:49:19.758843] [0x00007fbe766b0800] [info]    At time: 0.04 position: 0.0166281  1.43541e-05  0.159996
[2026-03-15 17:49:19.773935] [0x00007fbe766b0800] [info]    At time: 0.042 position: 0.0174651  2.02247e-05  0.15999
[2026-03-15 17:49:19.791439] [0x00007fbe766b0800] [info]    At time: 0.044 position: 0.0183026  2.4407e-05  0.159992
[2026-03-15 17:49:19.806636] [0x00007fbe766b0800] [info]    At time: 0.046 position: 0.0191408  2.38908e-05  0.159993
[2026-03-15 17:49:19.823233] [0x00007fbe766b0800] [info]    At time: 0.048 position: 0.0199796  2.83747e-05  0.159994
[2026-03-15 17:49:19.839922] [0x00007fbe766b0800] [info]    At time: 0.05 position: 0.0208194  3.02409e-05  0.159994
[2026-03-15 17:49:19.857106] [0x00007fbe766b0800] [info]    At time: 0.052 position: 0.0216601  3.2111e-05  0.159996
[2026-03-15 17:49:19.874365] [0x00007fbe766b0800] [info]    At time: 0.054 position: 0.0225017  3.56516e-05  0.159994
[2026-03-15 17:49:19.889853] [0x00007fbe766b0800] [info]    At time: 0.056 position: 0.0233442  3.70565e-05  0.159993
[2026-03-15 17:49:19.907112] [0x00007fbe766b0800] [info]    At time: 0.058 position: 0.0241875  3.91014e-05  0.159997
[2026-03-15 17:49:19.923881] [0x00007fbe766b0800] [info]    At time: 0.06 position: 0.0250316  3.76584e-05  0.159992
[2026-03-15 17:49:19.940460] [0x00007fbe766b0800] [info]    At time: 0.062 position: 0.0258765  3.69224e-05  0.159992
[2026-03-15 17:49:19.957294] [0x00007fbe766b0800] [info]    At time: 0.064 position: 0.0267219  3.70038e-05  0.159996
[2026-03-15 17:49:19.974156] [0x00007fbe766b0800] [info]    At time: 0.066 position: 0.0275681  3.7808e-05  0.159993
...
@endcode


