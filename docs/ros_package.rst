.. _chapter-ros-package:

===========
ROS Package
===========
We provided ROS Package example for running OpenVSLAM on ROS framework.

.. NOTE ::

    Please build OpenVSLAM with **OpenCV not 4 but 3** if you plan on using ROS Package.

.. NOTE ::

    This example does not suppourt macOS.

.. _section-installation:

Installation
============

Requirements
^^^^^^^^^^^^

* `ROS <http://wiki.ros.org/>`_ : version Kinetic or later

* :ref:`OpenVSLAM <chapter-installation>` : built with **OpenCV not 4 but 3**

* `image_transport <http://wiki.ros.org/image_transport>`_ : Required by this ROS Package example.

* `cv_bridge <http://wiki.ros.org/cv_bridge>`_ : built with the same version of OpenCV as OpenVSLAM **Please build from source** .

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Installing for Linux
--------------------

Tested for **Ubuntu 16.04**.

Please install the following dependencies.

* ROS : follow `Installation  <http://wiki.ros.org/ROS/Installation>`_.

* OpenVSLAM : follow :ref:`Installation <chapter-installation>`.

.. NOTE ::

    Please build OpenVSLAM with it if you plan on using PangolinViewer or SocketViewer on ROS Package.

Install the dependencies via ``apt``.

.. code-block:: bash

    apt update -y
    apt install ros-${ROS_DISTRO}-image-transport


Download the source of cv_bridge.

.. code-block:: bash

    cd /path/to/openvslam/ros
    git clone -b ${ROS_DISTRO} https://github.com/ros-perception/vision_opencv.git
    cp -r vision_opencv/cv_bridge src/
    rm -rf vision_opencv

.. NOTE ::

    It will be used from the source even if cv_bridge is installed via apt.

Build Instructions
^^^^^^^^^^^^^^^^^^

When building with support for PangolinViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=ON`` and ``-DUSE_SOCKET_PUBLISHER=OFF`` like :ref:`build of OpenVSLAM <section-build-unix>`.


.. code-block:: bash

    cd /path/to/openvslam/ros
    catkin_make \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DUSE_PANGOLIN_VIEWER=ON \
        -DUSE_SOCKET_PUBLISHER=OFF \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DBOW_FRAMEWORK=DBoW2

When building with support for SocketViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=OFF`` and ``-DUSE_SOCKET_PUBLISHER=ON`` like :ref:`build of OpenVSLAM <section-build-unix>`.


.. code-block:: bash

    cd /path/to/openvslam/ros
    export OpenVSLAM_DIR=$(cd ../ && pwd)

    catkin_make \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DUSE_PANGOLIN_VIEWER=OFF \
        -DUSE_SOCKET_PUBLISHER=ON \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DBOW_FRAMEWORK=DBoW2

.. _section-example:

Example
=======

Run core program required for ROS-based system.

.. code-block:: bash

    roscore

.. NOTE ::
**roscore** and **rosrun** command remains running. Please leave it run.

Publisher
^^^^^^^^^

Subscriber
^^^^^^^^^^
Subscriber continually receives images. Please do any one of the following.

.. NOTE ::

    Please set option arguments like the same as :ref:`OpenVSLAM Example <chapter-example>`.


Tracking and Mapping
--------------------

We provide an example snippet for visual SLAM.
The source code is placed at ``./ros/src/openvslam/src/run_slam.cc``.

.. code-block:: bash

    source /path/to/openvslam/ros/devel/setup.bash
    rosrun openvslam run_slam \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -c /path/to/aist_living_lab_1/config.yaml

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``./ros/src/openvslam/src/run_localization.cc``.


.. code-block:: bash

    source /path/to/openvslam/ros/devel/setup.bash
    rosrun openvslam run_localization \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -c /path/to/aist_living_lab_1/config.yaml \
        --map-db /path/to/map.msg
