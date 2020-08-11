.. _chapter-docker:

=================
Running on Docker
=================


.. _section-instructions-for-pangolinviewer:

Instructions for PangolinViewer
===============================

``Dockerfile.desktop`` can be used for easy installation.
This chapter provides instructions on building and running examples with PangolinViewer support using Docker.

The instructions are tested on Ubuntu 16.04 and 18.04.
Docker for Mac are NOT supported due to OpenGL forwarding.

Note that **docker host machines with NVIDIA graphics cards are NOT officially supported yet.**

.. NOTE ::

    If you plan on using a machine with NVIDIA graphics card(s), please use `nvidia-docker2 <https://github.com/NVIDIA/nvidia-docker>`_ and the version 390 or later of NVIDIA driver.
    These examples depend on X11 forwarding with OpenGL for visualization.
    Note that our ``Dockerfile.desktop`` is **NOT** compatible with nvidia-docker1.

If the viewer cannot be lanched at all or you are using macOS, please :ref:`install the dependencies manually <chapter-installation>` or use :ref:`the docker images for SocketViewer <section-instructions-for-socketviewer>`.

Building Docker Image
^^^^^^^^^^^^^^^^^^^^^

Execute the following commands:

.. code-block:: bash

    cd /path/to/openvslam
    docker build -t openvslam-desktop -f Dockerfile.desktop .


You can accelerate the build of the docker image with ``--build-arg NUM_THREADS=<number of parallel builds>`` option. For example:

.. code-block:: bash

    # building the docker image with four threads
    docker build -t openvslam-desktop -f Dockerfile.desktop . --build-arg NUM_THREADS=4

Starting Docker Container
^^^^^^^^^^^^^^^^^^^^^^^^^

In order to enable X11 forwarding, supplemental options (``-e DISPLAY=$DISPLAY`` and ``-v /tmp/.X11-unix/:/tmp/.X11-unix:ro``) are needed for ``docker run``.

.. code-block:: bash

    # before launching the container, allow display access from local users
    xhost +local:
    # launch the container
    docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro openvslam-desktop

.. NOTE ::

    Additional option ``--runtime=nvidia`` is needed if you use NVIDIA graphics card(s).  
    If you're using Docker with **Native GPU Support** then the options are ``--gpus all``.
    Please see `here <https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)#usage>`_ for more details.


After launching the container, the shell interface will be launched in the docker container.

.. code-block:: bash

    root@ddad048b5fff:/openvslam/build# ls
    lib                     run_image_slam          run_video_slam
    run_euroc_slam          run_kitti_slam          run_tum_slam
    run_image_localization  run_video_localization

See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.

.. NOTE ::

    If the viewer does not work, please :ref:`install the dependencies manually <section-dependencies>` on your host machine or use :ref:`the docker images for SocketViewer <section-instructions-for-socketviewer>` instead.

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.


.. _section-instructions-for-socketviewer:

Instructions for SocketViewer
=============================

``Dockerfile.socket`` and ``viewer/Dockerfile`` can be used for easy installation.
This chapter provides instructions on building and running examples with SocketViewer support using Docker.

Building Docker Images
^^^^^^^^^^^^^^^^^^^^^^

Docker Image of OpenVSLAM
`````````````````````````

Execute the following commands:

.. code-block:: bash

    cd /path/to/openvslam
    docker build -t openvslam-socket -f Dockerfile.socket .


You can accelerate the build of the docker image with ``--build-arg NUM_THREADS=<number of parallel builds>`` option. For example:

.. code-block:: bash

    # building the docker image with four threads
    docker build -t openvslam-socket -f Dockerfile.socket . --build-arg NUM_THREADS=4

Docker Image of Server
``````````````````````

Execute the following commands:

.. code-block:: bash

    cd /path/to/openvslam
    cd viewer
    docker build -t openvslam-server .

Starting Docker Containers
^^^^^^^^^^^^^^^^^^^^^^^^^^

On Linux
`````````````````````

Launch the server container and access to it with the web browser in advance.
Please specify ``--net=host`` in order to share the network with the host machine.

.. code-block:: bash

    $ docker run --rm -it --name openvslam-server --net=host openvslam-server
    WebSocket: listening on *:3000
    HTTP server: listening on *:3001

After launching, access to ``http://localhost:3001/`` with the web browser.

Next, launch the container of OpenVSLAM.
The shell interface will be launched in the docker container.

.. code-block:: bash

    $ docker run --rm -it --name openvslam-socket --net=host openvslam-socket
    root@hostname:/openvslam/build#

See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.

On macOS
`````````````````````

Launch the server container and access to it with the web browser in advance.
Please specify ``-p 3001:3001`` for port-forwarding.

.. code-block:: bash

    $ docker run --rm -it --name openvslam-server -p 3001:3001 openvslam-server
    WebSocket: listening on *:3000
    HTTP server: listening on *:3001

After launching, access to ``http://localhost:3001/`` with the web browser.

Then, inspect the container's IP address and append the ``SocketPublisher.server_uri`` entry to the YAML config file of OpenVSLAM.

.. code-block:: bash

    # inspect the server's IP address
    $ docker inspect openvslam-server | grep -m 1 \"IPAddress\" | sed 's/ //g' | sed 's/,//g'
    "IPAddress": "172.17.0.2"

.. code-block:: yaml

    # config file of OpenVSLAM

    ...

    #============================#
    # SocketPublisher Parameters #
    #============================#

    # append this entry
    SocketPublisher.server_uri: "http://172.17.0.2:3000"

Next, launch the container of OpenVSLAM.
The shell interface will be launched in the docker container.

.. code-block:: bash

    $ docker run --rm -it --name openvslam-socket openvslam-socket
    root@hostname:/openvslam/build#

| See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.
| Please don't forget to append ``SocketPublisher.server_uri`` entry to the ``config.yaml`` if you use the downloaded datasets in the tutorial.

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.

.. _section-directory-binding:

Bind of Directories
===================

If you need to access to any files and directories on a host machine from the container, bind directories between the host and the container using ``--volume`` or ``--mount`` option.
(See `the docker documentataion <https://docs.docker.com/engine/reference/commandline/run/>`_.)

For example:

.. code-block:: bash

    # launch a container of openvslam-desktop with --volume option
    $ docker run -it --rm --runtime=nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro \
        --volume /path/to/dataset/dir/:/dataset:ro \
        --volume /path/to/vocab/dir:/vocab:ro \
        openvslam-desktop
    # dataset/ and vocab/ are found at the root directory in the container
    root@0c0c9f115d74:/# ls /
    ...   dataset/   vocab/   ...

.. code-block:: bash

    # launch a container of openvslam-socket with --volume option
    $ docker run --rm -it --name openvslam-socket --net=host \
        --volume /path/to/dataset/dir/:/dataset:ro \
        --volume /path/to/vocab/dir:/vocab:ro \
        openvslam-socket
    # dataset/ and vocab/ are found at the root directory in the container
    root@0c0c9f115d74:/# ls /
    ...   dataset/   vocab/   ...
