Getting Started Guide Linux
===========================

In this guide we will show how to build Open1722 on a Linux system. We will also show how to run the unit tests and examples provided with Open1722.

Dependencies
------------

Before building Open1722 make sure you have installed the following software:

- CMake (version 3.20 or higher)
- CMocka (version 1.1.0 or higher, only required for running unit tests)

On debian/ubuntu you can install these dependencies with the following command:

.. code-block:: bash

    sudo apt-get install cmake libcmocka-dev

Building Open1722
------------------

The first step to build Open1722 is to generate the Makefile and build the project. This can be done with the following commands:

.. code-block:: bash

    mkdir build
    cd build
    cmake ..
    make

This builds the libraries libopen1722 containing all the data formats specified in the IEEE 1722 specification along with libopen1722custom which contains customized serialization formats which can be sent over IEEE 1722.

The examples can be built as follows:

.. code-block:: bash

    make examples

Unit tests
------------------

To build and execute available unit tests execute the following commands from the build directory:

.. code-block:: bash

    cmake ..
    make unittests
    make test

Installation
------------

In case you want to install Open1722 on your system run:

.. code-block:: bash

    make install

Cross-compilation
-----------------

To cross-compile for aarch64 (e.g. Raspberry Pi), you can use the provided toolchain file. Note that this requires a fresh build process with a new build folder.

.. code-block:: bash

    sudo apt install gcc-aarch64-linux-gnu libc6-dev-arm64-cross binutils-aarch64-linux-gnu
    cmake .. -DCMAKE_TOOLCHAIN_FILE=../aarch64.toolchain
    make

