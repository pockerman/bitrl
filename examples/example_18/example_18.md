\page bitrl_example_18 bitrl Example 18: DQN algorithm on Gridworld



In this example, we will train an agent so that it learns to navigate itself in a grid.
Specifically, we will be using the ```Gridworld``` environment from the book <a href="https://www.manning.com/books/deep-reinforcement-learning-in-action">Deep Reinforcement Learning in Action</a>.
The environment is the class \ref bitrl::envs::grid_world::Gridworld "Gridworld".

We will use the DQN algorithm, see <a href="https://www.manning.com/books/deep-reinforcement-learning-in-action">Deep Reinforcement Learning in Action</a> and references therein,
in order to train our agent. We will use a static environment configuration in this example something that makes this problem a lot easier to work on.
This example uses the same model as is done in the <a href="https://www.manning.com/books/deep-reinforcement-learning-in-action">Deep Reinforcement Learning in Action</a> 
book so you may also want to follow the code therein.

## Driver code

\include example_18/example_18.cpp

Running the code above produces the following output:

@code{.cpp}
[2025-06-28 09:22:40.880018] [0x00007f2d122e96c0] [info]    Starting agent training...
[2025-06-28 09:22:40.880034] [0x00007f2d122e96c0] [info]    Number of episodes to train: 1000
[2025-06-28 09:22:40.880258] [0x00007f2d122e96c0] [info]    Creating the environment...
[2025-06-28 09:22:40.880571] [0x00007f2d122e96c0] [info]    Done...
[2025-06-28 09:22:40.880577] [0x00007f2d122e96c0] [info]    Environment name: Gridworld
[2025-06-28 09:22:40.880583] [0x00007f2d122e96c0] [info]    Number of actions available: 4
[2025-06-28 09:22:40.880589] [0x00007f2d122e96c0] [info]    Number of states available: 16
[2025-06-28 09:22:40.906067] [0x00007f2d122e96c0] [info]    Starting epoch: 0
[2025-06-28 09:22:40.935573] [0x00007f2d122e96c0] [info]    	Action selected: 1

[2025-06-28 09:22:40.938906] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.959139] [0x00007f2d122e96c0] [info]    	Loss at epoch: 0.916412
[2025-06-28 09:22:40.959847] [0x00007f2d122e96c0] [info]    	Action selected: 3

[2025-06-28 09:22:40.959968] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.960585] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1.00951
[2025-06-28 09:22:40.960717] [0x00007f2d122e96c0] [info]    	Action selected: 3

[2025-06-28 09:22:40.960817] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.961204] [0x00007f2d122e96c0] [info]    	Loss at epoch: 0.909553
[2025-06-28 09:22:40.961301] [0x00007f2d122e96c0] [info]    	Action selected: 0

[2025-06-28 09:22:40.961388] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.961730] [0x00007f2d122e96c0] [info]    	Loss at epoch: 0.928638
[2025-06-28 09:22:40.961826] [0x00007f2d122e96c0] [info]    	Action selected: 2

[2025-06-28 09:22:40.961911] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.962236] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1.04332
[2025-06-28 09:22:40.962330] [0x00007f2d122e96c0] [info]    	Action selected: 3

[2025-06-28 09:22:40.962415] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.962731] [0x00007f2d122e96c0] [info]    	Loss at epoch: 0.94848
[2025-06-28 09:22:40.962823] [0x00007f2d122e96c0] [info]    	Action selected: 2

[2025-06-28 09:22:40.962907] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.963226] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1
[2025-06-28 09:22:40.963320] [0x00007f2d122e96c0] [info]    	Action selected: 2

[2025-06-28 09:22:40.963405] [0x00007f2d122e96c0] [info]    	Reward: -10
[2025-06-28 09:22:40.963723] [0x00007f2d122e96c0] [info]    	Loss at epoch: 100
[2025-06-28 09:22:40.963729] [0x00007f2d122e96c0] [info]    	Finishing epoch at step: 8
[2025-06-28 09:22:40.963746] [0x00007f2d122e96c0] [info]    Epoch finished...
[2025-06-28 09:22:40.964847] [0x00007f2d122e96c0] [info]    Starting epoch: 1
[2025-06-28 09:22:40.965032] [0x00007f2d122e96c0] [info]    	Action selected: 0

[2025-06-28 09:22:40.965122] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.965463] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1
[2025-06-28 09:22:40.965562] [0x00007f2d122e96c0] [info]    	Action selected: 1

[2025-06-28 09:22:40.965646] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.965969] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1
[2025-06-28 09:22:40.966065] [0x00007f2d122e96c0] [info]    	Action selected: 0

[2025-06-28 09:22:40.966149] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.966467] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1
[2025-06-28 09:22:40.966560] [0x00007f2d122e96c0] [info]    	Action selected: 0

[2025-06-28 09:22:40.966643] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.966957] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1
[2025-06-28 09:22:40.967051] [0x00007f2d122e96c0] [info]    	Action selected: 0

[2025-06-28 09:22:40.967134] [0x00007f2d122e96c0] [info]    	Reward: -1
[2025-06-28 09:22:40.967450] [0x00007f2d122e96c0] [info]    	Loss at epoch: 1
[2025-06-28 09:22:40.967544] [0x00007f2d122e96c0] [info]    	Action selected: 1

...
@endcode

The average per epoch loss is shown in the figure below

| ![average-per-epoch-loss](./average_epoch_loss.png) |
|:---------------------------------------------------:|
|         **Figure: Average loss per epoch.**         |

| ![average-per-epoch-loss](./average_epoch_reward.png) |
|:-----------------------------------------------------:|
|         **Figure: Average reward per epoch.**         |

