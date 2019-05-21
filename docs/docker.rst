.. _chapter-docker:

=================
Running on Docker
=================

``Dockerfile`` is provided to save effort for the installation.
This chapter describes how to build it and run example executables.

.. NOTE ::

    If you use a computer with the NVIDIA graphics card(s) as a host machine of Docker, please use `nvidia-docker2 <https://github.com/NVIDIA/nvidia-docker>`_.
    This is because the examples depend on X11 forwarding with OpenGL for visualization.
    Note that our ``Dockerfile`` is **NOT** compatible with nvidia-docker1.


.. _section-build-docker-image:

Building Docker Image
=====================

All you need is the following commands.

.. code-block:: bash

        git clone https://github.com/xdspacelab/openvslam
        cd openvslam
        docker build -t openvslam-desktop .


.. _section-start-docker-container:

Starting Docker Container
=========================

In order to enable X11 forwarding, supplemental options (``-e DISPLAY=$DISPLAY`` and ``-v /tmp/.X11-unix/:/tmp/.X11-unix:ro``) are needed for ``docker run``.

.. code-block:: bash

        docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro openvslam-desktop

.. NOTE ::

    If you use the NVIDIA graphics card(s), please insert the additional option ``--runtime=nvidia``.


After that, the shell interface will be launched in the docker container.

.. code-block:: bash

    root@ddad048b5fff:/openvslam/build# ls
    lib                     run_image_slam          run_movie_slam
    run_euroc_slam          run_kitti_slam          run_tum_slam
    run_image_localization  run_movie_localization

If you need to access to any files and directories on a host machine, please bind directories using ``--volume`` or ``--mount`` option.
(See `the docker documentataion <https://docs.docker.com/engine/reference/commandline/run/>`_.)

Here is an example.

.. code-block:: bash

    # launch a container
    $ docker run -it --rm --runtime=nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro \
        --volume /path/to/dataset/dir/:/dataset:ro \
        --volume /path/to/vocab/dir:/vocab:ro \
        openvslam-desktop
    # dataset/ and vocab/ are found in the root directory
    root@0c0c9f115d74:/# ls /
    ...   dataset/   vocab/   ...
