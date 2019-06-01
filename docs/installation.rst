.. _chapter-installation:

============
Installation
============


.. _section-get-source:

Source code
===========

The source code can be viewed from this `GitHub repository <https://github.com/xdspacelab/openvslam>`_.

Cloning the repository:

.. code-block:: bash

       git clone https://github.com/xdspacelab/openvslam

.. NOTE ::

    Users of **Docker for Ubuntu** can use :ref:`Docker <chapter-docker>` instead of preparing the dependencies manually.
    (Docker for Mac are not supported.)


.. _section-dependencies:

Dependencies
============

OpenVSLAM requires a **C++11-compliant** compiler.
It relies on several open-source libraries as shown below.

Requirements for OpenVSLAM
^^^^^^^^^^^^^^^^^^^^^^^^^^

* `Eigen <http://eigen.tuxfamily.org/>`_ : version 3.3.0 or later.

* `g2o <https://github.com/RainerKuemmerle/g2o>`_ : Please use the latest release. Tested on commit ID `9b41a4e <https://github.com/RainerKuemmerle/g2o/tree/9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a>`_.

* `SuiteSparse <http://faculty.cse.tamu.edu/davis/suitesparse.html>`_ : Required by g2o.

* `DBoW2 <https://github.com/shinsumicco/DBoW2>`_ : **Please use the custom version of DBoW2** released in `https://github.com/shinsumicco/DBoW2 <https://github.com/shinsumicco/DBoW2>`_.

* `yaml-cpp <https://github.com/jbeder/yaml-cpp>`_ : version 0.6.0 or later.

* `OpenCV <https://opencv.org/>`_ : version 3.4.0 or later.

.. NOTE ::

    OpenCV with GUI support is necessary for using the build-in viewer.

.. NOTE ::

    OpenCV with video support is necessary if you plan on using video files (e.g. ``.mp4``) as inputs.

Requirements for Pangolin Viewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| (**Recommended**)
| We provided a simple viewer implemented with `Pangolin <https://github.com/stevenlovegrove/Pangolin>`_.
| Please install the following dependencies if you plan on using the viewer.

* `Pangolin <https://github.com/stevenlovegrove/Pangolin>`_ : Please use the latest release. Tested on commit ID `ad8b5f8 <https://github.com/stevenlovegrove/Pangolin/tree/ad8b5f83222291c51b4800d5a5873b0e90a0cf81>`_.

* `GLEW <http://glew.sourceforge.net/>`_ : Required by Pangolin.

Requirements for Socket Publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| (**Optional**)
| If you plan to use the browser-based viewer, please see `here <https://github.com/xdspacelab/openvslam-viewer>`__.

Recommended
^^^^^^^^^^^

* `google-glog <https://github.com/google/glog>`_ : Used for stack-trace logger.


.. _section-prerequisites-unix:

Prerequisites for Unix
======================

.. NOTE ::

    In the following instruction, we assume that ``CMAKE_INSTALL_PREFIX`` is ``/usr/local``. If you want to install the libraries to the different location, set ``CMAKE_INSTALL_PREFIX`` to your environment and **set the environment variables accordingly**.

.. _section-linux:

Installing for Linux
^^^^^^^^^^^^^^^^^^^^

Tested for **Ubuntu 16.04**.

Install the dependencies via ``apt``.

.. code-block:: bash

    apt update -y
    apt upgrade -y --no-install-recommends
    # basic dependencies
    apt install -y build-essential pkg-config cmake git wget curl unzip
    # g2o dependencies
    apt install -y libatlas-base-dev libsuitesparse-dev
    # OpenCV dependencies
    apt install -y libgtk-3-dev
    apt install -y ffmpeg
    apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev
    # other dependencies
    apt install -y libyaml-cpp-dev libgoogle-glog-dev libgflags-dev 
    # Pangolin dependencies
    apt install -y libglew-dev

Download and install Eigen from source.

.. code-block:: bash

    cd /path/to/working/dir
    wget -q http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
    tar xf 3.3.4.tar.bz2
    rm -rf 3.3.4.tar.bz2
    cd eigen-eigen-5a0156e40feb
    mkdir -p build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ..
    make -j
    make install

Download, build and install OpenCV from source.

.. code-block:: bash

    cd /path/to/working/dir
    wget -q https://github.com/opencv/opencv/archive/3.4.0.zip
    unzip -q 3.4.0.zip
    rm -rf 3.4.0.zip
    cd opencv-3.4.0
    mkdir -p build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DENABLE_CXX11=ON \
        -DBUILD_DOCS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_JASPER=OFF \
        -DBUILD_OPENEXR=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
        -DWITH_EIGEN=ON \
        -DWITH_FFMPEG=ON \
        -DWITH_OPENMP=ON \
        ..
    make -j
    make install

Jump to :ref:`Common Installation Instructions <subsection-common-linux-macos>` for the next step.

.. _section-macos:

Installing for macOS
^^^^^^^^^^^^^^^^^^^^

Tested for **macOS High Sierra**.

Install the dependencies via ``brew``.

.. code-block:: bash

    brew update
    # basic dependencies
    brew install pkg-config cmake git
    # g2o dependencies
    brew install suite-sparse
    # OpenCV dependencies and OpenCV
    brew install eigen
    brew install ffmpeg
    brew install opencv
    # other dependencies
    brew install yaml-cpp glog gflags
    # Pangolin dependencies
    brew install glew

Jump to :ref:`Common Installation Instructions <subsection-common-linux-macos>` for the next step.

.. _subsection-common-linux-macos:

Common Installation Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Download, build and install **the custom DBoW2** from source.

.. code-block:: bash

    cd /path/to/working/dir
    git clone https://github.com/shinsumicco/DBoW2.git
    cd DBoW2
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ..
    make -j
    make install

Download, build and install g2o.

.. code-block:: bash

    cd /path/to/working/dir
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_CXX_FLAGS=-std=c++11 \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_UNITTESTS=OFF \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DG2O_USE_CHOLMOD=ON \
        -DG2O_USE_CSPARSE=ON \
        -DG2O_USE_OPENGL=OFF \
        -DG2O_USE_OPENMP=ON \
        ..
    make -j
    make install

Download, build and install Pangolin from source.

.. code-block:: bash

    cd /path/to/working/dir
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ..
    make -j
    make install


.. _section-build-unix:

Build Instructions
==================

When building with support for Pangolin Viewer.

.. code-block:: bash

    cd /path/to/openvslam
    git submodule init
    git submodule update
    mkdir build && cd build
    cmake \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DUSE_PANGOLIN_VIEWER=ON \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DBOW_FRAMEWORK=DBoW2 \
        -DBUILD_TESTS=ON \
        ..
    make -j

.. NOTE ::

    If ``cmake`` cannot find any dependencies, set the environment variables directly.
    For example, when ``CMAKE_INSTALL_PREFIX`` is ``/usr/local``:

    - ``Eigen3_DIR=/usr/local/share/eigen3/cmake``
    - ``OpenCV_DIR=/usr/local/share/OpenCV``
    - ``DBoW2_DIR=/usr/local/lib/cmake/DBoW2``
    - ``G2O_ROOT=/usr/local``
    - ``Pangolin_DIR=/usr/local/lib/cmake/Pangolin``

After building, check to see if it was successfully built by executing ``./build/run_kitti_slam -h``.

.. code-block:: bash

    $ ./build/run_kitti_slam -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -d, --data-dir arg     directory path which contains dataset
    -s, --setting arg      setting file path
    --frame-skip arg (=1)  interval of frame skipB
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode
    --eval-log             store trajectory and tracking times for evaluation
    --map-db arg           store a map database at this path after SLAM

.. NOTE ::

    If OpenVSLAM terminates abnormaly, rebuild g2o and OpenVSLAM with ``-DBUILD_WITH_MARCH_NATIVE=OFF`` option for ``cmake`` configulation.
