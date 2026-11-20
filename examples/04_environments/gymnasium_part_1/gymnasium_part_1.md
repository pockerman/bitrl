\page bitrl_example_gymnasium_part_1 Using Gymnasium environments (Part 1)

In this example we will see how to interact with  <a href="https://gymnasium.farama.org/index.html">Gymnasium</a> environments and 
specifically how to create an interact with <a href="https://gymnasium.farama.org/environments/toy_text/frozen_lake/">FroznLake</a> environment.

In _bitrl_, Gymnasium-based environments are interacted over a REST-like API maintained here: <a href="https://github.com/pockerman/bitrl-rest-api">bitsim-envs-api</a>.
_bitrl_ itself implements classes that hide this interaction from the client code. 
In general, environment classes in _bitrl_, have to implement the \ref bitrl::envs::EnvBase "bitrl::envs::EnvBase" API.

In this example we will use the \ref bitrl::envs::gymnasium::FrozenLake "bitrl::envs::gymnasium::FrozenLake"  
class. This is a template class, see the example below, that itself inherits from  \ref bitrl::envs::gymnasium::GymnasiumEnvBase "bitrl::envs::gymnasium::GymnasiumEnvBase"
class.

Below is the driver code. 

\include gymnasium_part_1/gymnasium_part_1.cpp

In order to run the example you will need an instance of the <a href="https://github.com/pockerman/bitrl-rest-api">bitsim-envs-api</a> server running
on your machine listening at port 8001. Note the actual example also shows how to use \ref bitrl::envs::gymnasium::Taxi "bitrl::envs::gymnasium::Taxi",
\ref bitrl::envs::gymnasium::CliffWorld "bitrl::envs::gymnasium::CliffWorld" and \ref bitrl::envs::gymnasium::BlackJack "bitrl::envs::gymnasium::BlackJack"
environments.
