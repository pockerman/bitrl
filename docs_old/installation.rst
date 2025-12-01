Installation
============

This section describes how to install ``rlenvscpp``.

Dependencies
------------

The library has the following general dependencies

- A compiler that supports C++20 e.g. g++-11
- `Boost C++ <https://www.boost.org/>`_ 
- `CMake >= 3.10 <https://cmake.org/>`_
- `Gtest  <https://github.com/google/googletest>`_  (if configured with tests)
- `Eigen3 <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_
- `Ray Ray <https://docs.ray.io/en/master/index.html>`_ (if configured with Ray)


``rlenvscpp`` also incorporates, see ``(src/extern)``, the following libraries:

- `HTTPRequest <https://github.com/elnormous/HTTPRequest>`_
- `nlohmann/json <https://github.com/nlohmann/json>`_

.. note::

	Using the Gymnasium environments requires `Gymnasium <https://github.com/Farama-Foundation/Gymnasium/tree/main>`_ 
	installed on your machine. In addition, you need to install

	- `FastAPI <https://fastapi.tiangolo.com/>`_
	- `Uvicorn <https://www.uvicorn.org/>`_
	- `Pydantic <a href="https://docs.pydantic.dev/latest/>`_

	By installing the requirement under ``requirements.txt`` should set your Python environment  up correctly.


Build the code
-----------------

The usual CMake based installation process is used. Namely

.. code-block::

	mkdir build && cd build && cmake ..
	make install


You can toggle the following variables

- CMAKE_BUILD_TYPE (default is RELEASE)
- ENABLE_TESTS_FLAG (default is OFF)
- ENABLE_EXAMPLES_FLAG (default is OFF)
- ENABLE_DOC_FLAG (default is OFF)
- ENABLE_WEBOTS (default OFF)
- ENABLE_RAY (default OFF)

For example enabling the examples: 

.. code-block::

	cmake -DENABLE_EXAMPLES_FLAG=ON ..
	make install
	

.. note::

	If you choose to enable Ray in your application, you do not have to
	build the library. Rather follow example `Using ``rlenvscpp`` with Ray <examples/ray_example_2.html>`_

Run the unit tests
-------------------

You can execute all the tests by running the helper script 

.. code-block::

	./execute_tests.sh


Build the documentation
-----------------------

Building the documentation is done via `Sphinx <https://www.sphinx-doc.org/en/master/>`_ and there are 
extra dependencies you need to install. Namely,

- `Doxygen <https://www.doxygen.nl/>`_
- breathe
- sphinx==5.0.2
- breathe==4.35.0
- exhale==0.3.7
- sphinx_rtd_theme==3.0.2

In order to build the documentation locally execute:

.. code-block::
	
	cd docs
	sphinx-build -M html . outputdir


where ``outputdir`` indicates a path in your system where you want to install the documentation files.
Check the official Sphinx `Getting started <https://www.sphinx-doc.org/en/master/usage/quickstart.html>`_ guide
for more options.


Issues
-------

**Could not find ``boost_system``**

It is likely that you are missing the boost_system library with your local Boost installation. This may be the case
is you installed boost via a package manager. On a Ubuntu machine the following should resolve the issue

.. code-block::

	sudo apt-get update -y
	sudo apt-get install -y libboost-system-dev


**FastAPI throws 422 Unpocessable entity**

Typically, this is a problem with how the client (400-range error) specified the data
to be sent to the server. 