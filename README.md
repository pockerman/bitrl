# cuberl

_cuberl_ is a C++ library containing implementations of various reinforcement learning, filtering and planning algorithms.
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

### bitrl

cuberl is built on <a href="https://github.com/pockerman/bitrl">bitrl</a>, which has to be installed first, see its
<a href="https://github.com/pockerman/bitrl/blob/master/doc/install.md">install page</a>. Then tell CMake where it is:

```
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/bitrl/install/opt
cmake --build build --parallel
```

Without `-DCMAKE_INSTALL_PREFIX`, bitrl installs into `install/opt` in its source tree. Add the Chrono prefix
to `CMAKE_PREFIX_PATH` too if Chrono is not in a system prefix, separated by `;`.

### Enabling PyTorch and CUDA

_cuberl_ can be complied with CUDA and/or PyTorch support. If PyTorch has been compiled using CUDA support, then
you need to enable CUDA as well. In order to do so set the flag _USE_CUDA_ in the _CMakeLists.txt_ to _ON_.
_cuberl_ assumes that PyTorch is compiled with the C++11 ABI.


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

This issue may be occur when using the TensorBoardServer in _cuberl_.
This issue  is related with an issue with _protobuf_. See: https://stackoverflow.com/questions/72441758/typeerror-descriptors-cannot-not-be-created-directly for 
possible solutions.



