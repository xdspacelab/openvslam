.. _chapter-docker:

=================
Running on Docker
=================

``Dockerfile`` can be used for easy installation.
This chapter provides instructions on building and running examples using Docker.

.. NOTE ::

    Use `nvidia-docker2 <https://github.com/NVIDIA/nvidia-docker>`_ if you plan on using a machine with NVIDIA graphics card(s).
    These examples depend on X11 forwarding with OpenGL for visualization.
    Note that our ``Dockerfile`` is **NOT** compatible with nvidia-docker1.


.. _section-build-docker-image:

Building Docker Image
=====================

Execute the following commands:

.. code-block:: bash

    git clone https://github.com/xdspacelab/openvslam
    cd openvslam
    docker build -t openvslam-desktop .


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

    Additional option ``--runtime=nvidia`` is needed, if you use NVIDIA graphics card(s).


After launching the container, the shell interface will be launched in the docker container.

.. code-block:: bash

    root@ddad048b5fff:/openvslam/build# ls
    lib                     run_image_slam          run_movie_slam
    run_euroc_slam          run_kitti_slam          run_tum_slam
    run_image_localization  run_movie_localization

See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples.

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
