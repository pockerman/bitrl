\page bitrl_example_gymnasium_part_2 Using Gymnasium environments (Part 2)

In this example we will see how to interact with  <a href="https://gymnasium.farama.org/index.html">Gymnasium</a> environments and
specifically how to create an interact with <a href="https://gymnasium.farama.org/environments/classic_control/cart_pole/">CartPole</a> environment.

As already mentioned in  \ref bitrl_example_gymnasium_part_1, Gymnasium-based environments are interacted over a REST-like API maintained here: <a href="https://github.com/pockerman/bitrl-rest-api">bitsim-envs-api</a>.
_bitrl_ itself implements classes that hide this interaction from the client code.
In general, environment classes in _bitrl_, have to implement the \ref bitrl::envs::EnvBase "bitrl::envs::EnvBase" API.

In this example we will use the \ref bitrl::envs::gymnasium::CartPole "bitrl::envs::gymnasium::CartPole"  
class. This is a template class, see the example below, that itself inherits from  \ref bitrl::envs::gymnasium::GymnasiumEnvBase "bitrl::envs::gymnasium::GymnasiumEnvBase"
class.

Below is the driver code. 

\include gymnasium_part_2/gymnasium_part_2.cpp


In order to run the example you will need an instance of the <a href="https://github.com/pockerman/bitrl-rest-api">bitsim-envs-api</a> server running
on your machine listening at port 8001. Note the actual example also shows how to use \ref bitrl::envs::gymnasium::Pendulum "bitrl::envs::gymnasium::Pendulum",
\ref bitrl::envs::gymnasium::Acrobot "bitrl::envs::gymnasium::Acrobot" and \ref bitrl::envs::gymnasium::MountainCar "bitrl::envs::gymnasium::MountainCar"
environments.