\page bitrl_example_13 BitRL Example 13 Using chrono::ChSystemSMC

Example \ref bitrl_example_11 discussed _ChBody_. Specifically, how to create a _ChBodyEasyBox_.
Understanding rigid bodies is fundamental to robotics. In this example we will discuss _ChSystem_.
This is the cornerstone of a Chrono simulation. Most information in this example
can be found in the official doumentation here: <a href="https://api.projectchrono.org/9.0.0/simulation_system.html">Simulation system</a>.
A _ChSystem_ is an abstract class. The Chrono library provides the following subclasses:

- _ChSystemNSC_ for Non Smooth Contacts (NSC): in case of contacts a complementarity solver will take care of them using non-smooth dynamics; this is very efficient even with large time steps.
- _ChSystemSMC_ for SMooth Contacts (SMC): contacts are handled using penalty methods, i.e. contacts are deformable

Note that if there are no contacts or collisions in your system, it is indifferent to use _ChSystemNSC_ or _ChSystemSMC_.
In this example we will create and simulate a differential drive system using Chrono

@code{.cpp}
@endcode


