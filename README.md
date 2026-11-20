# bitrl

_bitrl_ is a C++ library containing implementations of various reinforcement learning, filtering and planning algorithms.
The following is an indicative list of examples. 
 

## Examples

### Introductory

- <a href="examples/intro/intro_example_1/intro_example_1.md">Monte Carlo integration</a>
- <a href="examples/intro/intro_example_2/intro_example_2.md">Using PyTorch C++ API Part 1</a>
- <a href="examples/intro/intro_example_3/intro_example_3.md">Using PyTorch C++ API Part 2</a>
- <a href="examples/intro/intro_example_4/intro_example_4.md">Using PyTorch C++ API Part 3</a>
- <a href="examples/intro/intro_example_6/intro_example_6.md">Toy Markov chain</a>
- <a href="examples/intro/intro_example_7/intro_example_7.md">Importance sampling</a>
- <a href="examples/intro/intro_example_8/intro_example_8.md">Vector addition with CUDA</a>

### Environments

- <a href="examples/envs/grid_world/grid_world.md">A grid world</a>
- <a href="examples/envs/connect2/connect2.md">Connect2</a>
- <a href="examples/envs/gymnasium_part_1/gymnasium_part_1.md">Using Gymnasium environments (Part 1)</a>
- <a href="examples/envs/gymnasium_part_2/gymnasium_part_2.md">Using Gymnasium environments (Part 2)</a>
- <a href="examples/envs/gymnasium_part_3/gymnasium_part_3.md">Using Gymnasium environments (Part 3)</a>
- <a href="examples/envs/gym_walk/gym_walk.md">Using GymWalk environment</a>
- <a href="examples/envs/acrobot_vector_env/acrobot_vector_env.md">A vector environment: Acrobot</a>
- <a href="examples/envs/tensorboard/tensorboard.md">Logging to Tensorboard</a>

### Robotics environments

These need _bitsim_ built with Chrono.

- <a href="examples/robotics/chrono_smc_system/chrono_smc_system.md">A bitsim robot as an RL environment</a>
- <a href="examples/robotics/chrono_rl_env/chrono_rl_env.md">Create an RL environment using Chrono</a>

### Reinforcement learning

- <a href="https://pockerman-py-cubeai.readthedocs.io/en/latest/ExamplesCpp/rl/rl_example_0.html">DummyAgent on  ```MountainCar-v0```</a>
- <a href="examples/rl/rl_example_2/rl_example_2.md">Multi-armed bandits</a>
- <a href="examples/rl/rl_example_6/rl_example_6.cpp">Iterative policy evaluation on ```FrozenLake```</a>
- <a href="examples/rl/rl_example_7/rl_example_7.cpp">Policy iteration on ```FrozenLake```</a>
- <a href="examples/rl/rl_example_8/rl_example_8.md">Value iteration on ```FrozenLake```</a>
- <a href="examples/rl/rl_example_9/rl_example_9.md">SARSA on ```CliffWalking```</a>
- <a href="examples/rl/rl_example_10/rl_example_10.md">Q-learning on ```CliffWalking```</a>
- <a href="examples/rl/rl_example_11/rl_example_11.md">A2C on ```CartPole```</a>
- <a href="examples/rl/rl_example_12/rl_example_12.md">DQN on ```Gridworld```</a>
- <a href="examples/rl/rl_example_15/rl_example_15.md">DQN on ```Gridworld``` with experience replay</a>
- <a href="examples/rl/rl_example_13/rl_example_13.md">REINFORCE algorithm on ```CartPole```</a>
- <a href="examples/rl/rl_example_14/rl_example_14.cpp">Expected SARSA on ```CliffWalking```</a>
- <a href="examples/example_15/example_15.cpp">Approximate Monte Carlo on ```MountainCar```</a>
- <a href="examples/rl_example_16/rl_example_16.md">Monte Carlo tree search on ```Taxi```</a>
- <a href="examples/rl/rl_example_18.cpp">Double Q-learning on  ```CartPole``` </a>
- <a href="examples/rl/rl_example_19/rl_example_19.cpp">First visit Monte Carlo on ```FrozenLake```</a>
- <a href="examples/rl/rl_example_20/rl_example_20.md">REINFORCE algorithm with baseline on ```CartPole```</a>
- <a href="examples/rl/rl_example_22/rl_example_22.md">PPO on ```LunarLander```</a>

## Installation

The cubeai library has a host of dependencies:

- A compiler that supports C++20 e.g. g++-11
- <a href="https://www.boost.org/">Boost C++</a> 
- <a href="https://cmake.org/">CMake</a> >= 3.6
- <a href="https://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a>
- <a href="https://github.com/google/googletest">Gtest</a> (if configured with tests)

In addition, the library also incorporates, see ```(include/cubeai/extern)```, the following libraries (you don't need to install these):

- <a href="https://github.com/elnormous/HTTPRequest">HTTPRequest</a>
- <a href="https://github.com/nlohmann/json">nlohmann/json</a>

### bitsim and libtorch

bitrl is built on <a href="https://github.com/pockerman/bitsim">bitsim</a>, which has to be installed first, see its
<a href="https://github.com/pockerman/bitsim/blob/master/doc/install.md">install page</a>. bitsim installs as the CMake
package `bitsim`. bitrl also needs <a href="https://pytorch.org/get-started/locally/">libtorch</a>, built with the
C++11 ABI, and TBB (`sudo apt-get install libtbb-dev`). Tell CMake where bitsim and libtorch are:

```
cmake -B build -DCMAKE_PREFIX_PATH="/path/to/bitsim/prefix;/path/to/libtorch"
cmake --build build --parallel 3
```

Give `--parallel` a number. Files that include libtorch take a lot of memory to compile, and an unlimited
build runs out of memory on a 16 GB machine.

Add the Chrono prefix to `CMAKE_PREFIX_PATH` too if bitsim was built with Chrono and Chrono is not in a system prefix.
`ci/install_bitsim.sh` and `ci/install_libtorch.sh` do both installs the way CI does.

The options are `ENABLE_EXAMPLES` (ON), `ENABLE_TESTS` (OFF), `USE_CUDA` (OFF; libtorch must be built with CUDA too),
`BITRL_LOG` (ON) and `BITRL_WERROR` (OFF), for example

```
cmake -B build -DCMAKE_PREFIX_PATH="..." -DENABLE_TESTS=ON
cmake --build build --parallel 3
ctest --test-dir build --output-on-failure
```

### Documentation dependencies

There are extra dependencies if you want to generate the documentation. Namely,

- <a href="https://www.doxygen.nl/">Doxygen</a>
- <a href="https://www.sphinx-doc.org/en/master/">Sphinx</a>
- <a href="https://github.com/readthedocs/sphinx_rtd_theme">sphinx_rtd_theme</a>
- <a href="https://github.com/breathe-doc/breathe">breathe</a>
- <a href="https://github.com/crossnox/m2r2">m2r2</a>

Note that if Doxygen is not found on your system CMake will skip this. On a Ubuntu/Debian based machine, you can install
Doxygen using

```bash
sudo apt-get install doxygen
```

Similarly, install ```sphinx_rtd_theme``` using

```bash
pip install sphinx_rtd_theme
```

Install ```breathe``` using

```bash
pip install breathe
```

Install ```m2r2``` using

```bash
pip install m2r2
```


## Issues

#### undefined reference to ```cudaLaunchKernelExC@libcudart.so.11.0```. 

You may want to check with ```nvidia-msi``` your CUDA Version and make sure it is compatible with the PyTorch library you are linking against

#### TypeError: Descriptors cannot be created directly.

This issue may be occur when using the TensorBoardServer in _bitrl_.
This issue  is related with an issue with _protobuf_. See: https://stackoverflow.com/questions/72441758/typeerror-descriptors-cannot-not-be-created-directly for 
possible solutions.



