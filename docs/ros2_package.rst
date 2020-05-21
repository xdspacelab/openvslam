.. _chapter-ros-package:

============
ROS2 Package
============

.. _section-installation:

Installation
============

Requirements
^^^^^^^^^^^^

* `ROS2 <https://index.ros.org/doc/ros2//>`_ : Please use the version ``dashing`` or later.

* :ref:`OpenVSLAM <chapter-installation>` : Please build it with **OpenCV 3.x**.

* `image_common <https://index.ros.org/r/image_common/github-ros-perception-image_common>`_ : Required by this ROS package examples.

* `vision_opencv <https://index.ros.org/r/vision_opencv/github-ros-perception-vision_opencv>`_ : Please build it with the same version of OpenCV used in OpenVSLAM. (**We recommend building it from source.**)

* `image_tools <https://index.ros.org/p/image_tools/#dashing>`_ : An optional requirement to use USB cameras.

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Tested for **Ubuntu 18.04**.

Please install the following dependencies.

* ROS2 : Please follow `Installation of ROS2 <https://index.ros.org/doc/ros2/Installation/>`_.

* OpenVSLAM : Please follow :ref:`Installation of OpenVSLAM <chapter-installation>`.

.. NOTE ::

    Please build OpenVSLAM with PangolinViewer or SocketViewer if you plan on using it for the examples.

Download repositories of ``image_common`` and ``vision_opencv``.

.. code-block:: bash

    cd /path/to/openvslam/ros/2/src
    git clone -b dashing --single-branch https://github.com/ros-perception/image_common.git
    git clone -b ros2 --single-branch https://github.com/ros-perception/vision_opencv.git

For using USB cam as a image source, donload a repository of ``demos`` and pick ``image_tools`` module.

.. code-block:: bash

    cd /path/to/openvslam/ros/2
    git clone https://github.com/ros2/demos.git
    cp -r demos/image_tools src/
    rm -rf demos

Build Instructions
^^^^^^^^^^^^^^^^^^

When building with support for PangolinViewer, you have not specify any arguments.

.. code-block:: bash

    cd /path/to/openvslam/ros/2
    colcon buld --symlink-install

The SocketViewer is not available right now. We will provide this feature soon.

Examples
========

Publisher
^^^^^^^^^

Publishers continually broadcast images as a ROS topic.
Please execute one of the following command snippets in the new terminal.

Publish a Video File
--------------------

For using video files (e.g. ``.mp4``) for visual SLAM or localization.

.. code-block:: bash

    source /path/to/openvslam/ros/2/install/setup.bash
    ros2 run publisher video -m /path/to/video.mp4

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    ros2 run image_transport republish \
        raw in:=/video/image_raw raw out:=/camera/image_raw


Publish a Image Sequence
------------------------

For using image sequences for visual SLAM or localization.

.. code-block:: bash

    source /path/to/openvslam/ros/2/install/setup.bash
    ros2 run publisher image -i /path/to/images/

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    ros2 run image_transport republish \
        raw in:=/video/image_raw raw out:=/camera/image_raw

Publish Images Captured by a USB Camera
------------------------------

For using a standard USB camera for visual SLAM or localization.

.. code-block:: bash

    ros2 run image_tools cam2image

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    rosrun image_transport republish \
        raw in:=image raw out:=/camera/image_raw

Subscriber
^^^^^^^^^^

Subscribers continually receive images.
Please execute one of the following command snippets in the new terminal.

.. NOTE ::

    Option arguments are the same as :ref:`the examples of OpenVSLAM <chapter-example>`.

Tracking and Mapping
--------------------

We provide an example snippet for visual SLAM.
The source code is placed at ``./openvslam/ros/src/openvslam/src/run_slam.cc``.

.. code-block:: bash

    source /path/to/openvslam/ros/2/install/setup.bash
    ros2 run openvslam run_slam \
        -v /path/to/orb_vocab.dbow2 \
        -c /path/to/config.yaml

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``./ros/src/openvslam/src/run_localization.cc``.

.. code-block:: bash

    source /path/to/openvslam/ros/2/install/setup.bash
    ros2 run openvslam run_localization \
        -v /path/to/orb_vocab.dbow2 \
        -c /path/to/config.yaml \
        --map-db /path/to/map.msg
