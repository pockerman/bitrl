# bitrl

_bitrl_ is a C++ library containing implementations of various reinforcement learning, filtering and planning algorithms.
The examples, with a page for each, are listed in [examples/examples.md](examples/examples.md). Some of them: 
 

## Examples

### Introductory

- <a href="examples/example_1/example_1.md">Monte Carlo integration</a>
- <a href="examples/example_2/example_2.md">Using the PyTorch C++ API (Part 1)</a>
- <a href="examples/example_3/example_3.md">Using the PyTorch C++ API (Part 2)</a>
- <a href="examples/example_4/example_4.md">Using the PyTorch C++ API (Part 3)</a>
- <a href="examples/example_6/example_6.md">Toy Markov chain</a>
- <a href="examples/example_7/example_7.md">Importance sampling</a>
- <a href="examples/example_8/example_8.md">Vector addition with CUDA</a>
- <a href="examples/example_9/example_9.md">Discounted returns</a>

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

- <a href="examples/example_10/example_10.md">Multi-armed bandits</a>
- <a href="examples/example_11/example_11.md">Multi-armed bandits with a soft-max policy</a>
- <a href="examples/example_12/example_12.md">Iterative policy evaluation on ```FrozenLake```</a>
- <a href="examples/example_13/example_13.md">Policy iteration on ```FrozenLake```</a>
- <a href="examples/example_14/example_14.md">Value iteration on ```FrozenLake```</a>
- <a href="examples/example_15/example_15.md">SARSA on ```CliffWorld```</a>
- <a href="examples/example_16/example_16.md">Q-learning on ```CliffWorld```</a>
- <a href="examples/example_17/example_17.md">A2C on ```CartPole```</a>
- <a href="examples/example_18/example_18.md">DQN on ```Gridworld```</a>
- <a href="examples/example_21/example_21.md">DQN on ```Gridworld``` with experience replay</a>
- <a href="examples/example_19/example_19.md">REINFORCE on ```CartPole```</a>
- <a href="examples/example_23/example_23.md">REINFORCE with baseline on ```CartPole```</a>
- <a href="examples/example_22/example_22.md">First visit Monte Carlo on ```FrozenLake```</a>
- <a href="examples/example_25/example_25.md">PPO on ```LunarLander```</a>

## Installation

bitrl needs

- a compiler that supports C++20, for example g++ 11
- <a href="https://cmake.org/">CMake</a> 3.20 or later
- <a href="https://www.boost.org/">Boost</a> and <a href="https://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a>
- <a href="https://github.com/google/googletest">GoogleTest</a>, if it is built with the tests

HTTPRequest and nlohmann/json come with bitsim.

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

### Install

```
cmake -B build -DCMAKE_PREFIX_PATH="..." -DCMAKE_INSTALL_PREFIX=$HOME/opt/bitrl
cmake --build build --parallel 3
cmake --install build
```

installs `lib/libbitrllib.so`, the headers in `include/bitrl/` and the CMake package in
`lib/cmake/bitrl/`. Another project then uses bitrl with

```
find_package(bitrl 5 REQUIRED)
target_link_libraries(my_target PRIVATE bitrl::bitrl)
```

and `-DCMAKE_PREFIX_PATH` with the prefixes of bitrl, bitsim and libtorch. The package finds bitsim,
libtorch and TBB itself. `ci/check_install.sh` checks that an installed bitrl works this way.

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



