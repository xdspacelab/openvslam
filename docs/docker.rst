.. _chapter-docker:

=================
Running on Docker
=================

``Dockerfile`` can be used for easy installation.
This chapter provides instructions on building and running examples using Docker.

Note that **docker host machines with NVIDIA graphics cards are NOT officially supported yet.**

.. NOTE ::

    If you plan on using a machine with NVIDIA graphics card(s), please use `nvidia-docker2 <https://github.com/NVIDIA/nvidia-docker>`_ and the version 390 or later of NVIDIA driver.
    These examples depend on X11 forwarding with OpenGL for visualization.
    Note that our ``Dockerfile`` is **NOT** compatible with nvidia-docker1.

| The instructions are tested on Ubuntu 16.04 and 18.04.
| Docker for Mac are not supported due to OpenGL forwarding. Please :ref:`install the dependencies manually <section-macos>` if you are using macOS.

.. _section-build-docker-image:

Building Docker Image
=====================

Execute the following commands:

.. code-block:: bash

    git clone https://github.com/xdspacelab/openvslam
    cd openvslam
    docker build -t openvslam-desktop .


You can accelerate the build of the docker image with ``--build-arg NUM_THREADS=<number of parallel builds>`` option. For example:

.. code-block:: bash

    # building the docker image with four threads
    docker build -t  openvslam-desktop . --build-arg NUM_THREADS=4


.. _section-start-docker-container:

Starting Docker Container
=========================

In order to enable X11 forwarding, supplemental options (``-e DISPLAY=$DISPLAY`` and ``-v /tmp/.X11-unix/:/tmp/.X11-unix:ro``) are needed for ``docker run``.

.. code-block:: bash

    # before launching the container, allow display access from local users
    xhost +local:
    # launch the container
    docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro openvslam-desktop

.. NOTE ::

    Additional option ``--runtime=nvidia`` is needed if you use NVIDIA graphics card(s).


After launching the container, the shell interface will be launched in the docker container.

.. code-block:: bash

    root@ddad048b5fff:/openvslam/build# ls
    lib                     run_image_slam          run_video_slam
    run_euroc_slam          run_kitti_slam          run_tum_slam
    run_image_localization  run_video_localization

See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples.

.. NOTE ::

    If the viewer does not work, please :ref:`install the dependencies manually <section-dependencies>` on your host machine and try :ref:`Tutorial <chapter-simple-tutorial>`.

If you need to access to any files and directories on a host machine from the container, bind directories between the host and the container using ``--volume`` or ``--mount`` option.
(See `the docker documentataion <https://docs.docker.com/engine/reference/commandline/run/>`_.)

For example:

.. code-block:: bash

    # launch a container
    $ docker run -it --rm --runtime=nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro \
        --volume /path/to/dataset/dir/:/dataset:ro \
        --volume /path/to/vocab/dir:/vocab:ro \
        openvslam-desktop
    # dataset/ and vocab/ are found in the root directory
    root@0c0c9f115d74:/# ls /
    ...   dataset/   vocab/   ...
