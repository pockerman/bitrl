# bitrl-doc
_bitrl_ is an effort to provide implementations and wrappers of environments suitable for training reinforcement learning agents
using  C++.

The following is an example how to use the
_FrozenLake_   environment from <a href="https://github.com/Farama-Foundation/Gymnasium/tree/main">Gymnasium</a>.

@code
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <any>

namespace example_1 {
const std::string SERVER_URL = "http://0.0.0.0:8001/api";
using bitrl::envs::gymnasium::FrozenLake;
using bitrl::envs::RESTApiServerWrapper;

void test_frozen_lake(const RESTApiServerWrapper& server) {

     FrozenLake<4> env(server);
     std::cout << "Environment URL: " << env.get_url() << std::endl;
     
     std::unordered_map<std::string, std::any> make_ops;
     make_ops.insert({"is_slippery", false});
     
     std::unordered_map<std::string, std::any> reset_ops;
     reset_ops.insert({"seed", static_cast<uint_t>(42)});

     env.make("v1", make_ops, reset_ops);
}
} // namespace example_1

int main() {
RESTApiServerWrapper server(SERVER_URL, true);
example_1::test_frozen_lake(server);
return 0;
}
@endcode


Gymnasium environments exposed over a REST like API can be found at: <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a>.
Various RL algorithms using the environments can be found at <a href="https://github.com/pockerman/cuberl/tree/master">cuberl</a>.

## Dependencies

_bitrl_ has a number of dependencies assumed to be installed under usual destination on a system:

- Boost
- Eigen3
- Blas
- OpenCV
- PahoMqttCpp

## Installation

The usual _cmake_ installation/build can be used:

@code
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/path/where/bitrl/should/be/installed/to ..
make install -j4
@endcode

## Examples

