.. _chapter-example:

=======
Example
=======

We provide example code snippets to run OpenVSLAM with variety of datasets.

.. _section-example-movie:

SLAM with a movie
=================

Tracking and Mapping
^^^^^^^^^^^^^^^^^^^^

We provide a example snippet to use a movie (e.g. ``.mp4``) for visual SLAM.
The source code is placed at ``./example/run_movie_slam.cc``.
The following options are allowed.

.. code-block::

    $ ./run_movie_slam -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -m, --movie arg        movie file path
    -s, --setting arg      setting file path
    --mask arg             mask image path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode
    --eval-log             store trajectory and tracking times for evaluation
    --map-db arg           store a map database at this path after SLAM

| The camera that capture the movie must be calibrated. According to the calibrated parameters, please create a config file (``.yaml``).
| We provide a vocabulary file for DBoW2 at `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__. You can use ``orb_vocab.dbow2`` in the zip file.

Localization
^^^^^^^^^^^^

We provide a example snippet to use a movie (e.g. ``.mp4``) for localization based on a prebuilt map.
The source code is placed at ``./example/run_movie_localization.cc``.
The following options are allowed.

.. code-block::

    $ ./run_movie_localization -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -m, --movie arg        movie file path
    -s, --setting arg      setting file path
    -d, --map-db arg       path to a prebuilt map database
    --mapping              perform mapping as well as localization
    --mask arg             mask image path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode

| The camera that capture the movie must be calibrated. According to the calibrated parameters, please create a config file (``.yaml``).
| We provide a vocabulary file for DBoW2 at `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__. You can use ``orb_vocab.dbow2`` in the zip file.

You can create a map database file by running one of the ``run_****_slam`` executables with ``--map-db map_file_name.msg`` option.

.. _section-example-image-sequence:

SLAM with an image sequence
===========================

Tracking and Mapping
^^^^^^^^^^^^^^^^^^^^

We provide a example snippet to use an image sequence for visual SLAM.
The source code is placed at ``./example/run_image_slam.cc``.
The following options are allowed.

.. code-block::

    $ ./run_image_slam -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -i, --img-dir arg      directory path which contains images
    -s, --setting arg      setting file path
    --mask arg             mask image path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode
    --eval-log             store trajectory and tracking times for evaluation
    --map-db arg           store a map database at this path after SLAM

| The camera that capture the movie must be calibrated. According to the calibrated parameters, please create a config file (``.yaml``).
| We provide a vocabulary file for DBoW2 at `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__. You can use ``orb_vocab.dbow2`` in the zip file.

Localization
^^^^^^^^^^^^

We provide a example snippet to use an image sequence for localization based on a prebuilt map.
The source code is placed at ``./example/run_image_localization.cc``.
The following options are allowed.

.. code-block::

    $ ./run_image_localization -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -i, --img-dir arg      directory path which contains images
    -s, --setting arg      setting file path
    -d, --map-db arg       path to a prebuilt map database
    --mapping              perform mapping as well as localization
    --mask arg             mask image path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode

| The camera that capture the movie must be calibrated. According to the calibrated parameters, please create a config file (``.yaml``).
| We provide a vocabulary file for DBoW2 at `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__. You can use ``orb_vocab.dbow2`` in the zip file.

You can create a map database file by running one of the ``run_****_slam`` executables with ``--map-db map_file_name.msg`` option.

.. _section-example-standard-datasets:

SLAM with standard datasets
===========================

.. _subsection-example-kitti:

KITTI Odometry dataset
^^^^^^^^^^^^^^^^^^^^^^

`KITTI Odometry dataset <http://www.cvlibs.net/datasets/kitti/>`_ is a benchmarking dataset for monocular and stereo visual odometry and lidar odometry that captured with car-mounted devices.
We provide an example source code to run monocular and stereo visual SLAM with this dataset.
The source code is placed at ``./example/run_kitti_slam.cc``.

Start by downloading the dataset from `here <http://www.cvlibs.net/datasets/kitti/eval_odometry.php>`__.
Please donwload the grayscale set (``data_odometry_gray.zip``).

After downloading and uncompressing it, you will find several sequences under ``sequences/`` directory.

.. code-block:: bash

    $ ls sequences/
    00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21

In addition, please download a vocabulary file for DBoW2 from `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__ and uncompress it.
You can find ``orb_vocab.dbow2`` in the zip file.

With this, you can run visual SLAM with KITTI Odometry dataset!

A setting file for each sequence is contained under ``./example/kitti/``.

If you have built examples with Pangolin Viewer support, a map viewer and frame viewer will be started soon after executing the following command.

.. code-block:: bash

    # at the build directory of OpenVSLAM
    $ ls
    ...
    run_kitti_slam
    ...
    # monocular SLAM with sequence 00
    $ ./run_kitti_slam \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -d /path/to/KITTI/Odometry/sequences/00/ \
        -s ../example/kitti/KITTI_mono_00-02.yaml
    # stereo SLAM with sequence 05
    $ ./run_kitti_slam \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -d /path/to/KITTI/Odometry/sequences/05/ \
        -s ../example/kitti/KITTI_stereo_04-12.yaml

The following options are allowed.

.. code-block::

    $ ./run_kitti_slam -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -d, --data-dir arg     directory path which contains dataset
    -s, --setting arg      setting file path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode
    --eval-log             store trajectory and tracking times for evaluation
    --map-db arg           store a map database at this path after SLAM

.. _subsection-example-euroc:

EuRoC MAV dataset
^^^^^^^^^^^^^^^^^

`EuRoC MAV dataset <https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets>`_ is a benchmarking dataset for monocular and stereo visual odometry that captured with drone-mounted devices.
We provide an example source code to run monocular and stereo visual SLAM with this dataset.
The source code is placed at ``./example/run_euroc_slam.cc``.

Start by downloading the dataset from `here <http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/>`__.
Please donwload the ``.zip`` file of a dataset you want to use.

After downloading and uncompressing it, you will find several directories under ``mav0/`` directory.

.. code-block:: bash

    $ ls mav0/
    body.yaml  cam0  cam1  imu0  leica0  state_groundtruth_estimate0

In addition, please download a vocabulary file for DBoW2 from `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__ and uncompress it.
You can find ``orb_vocab.dbow2`` in the zip file.

With this, you can run visual SLAM with EuRoC MAV dataset!

We provide the two setting files for EuRoC, ``./example/euroc/EuRoC_mono.yaml`` for monocular and ``./example/euroc/EuRoC_stereo.yaml`` for stereo.

If you have built examples with Pangolin Viewer support, a map viewer and frame viewer will be started soon after executing the following command.

.. code-block:: bash

    # at the build directory of OpenVSLAM
    $ ls
    ...
    run_euroc_slam
    ...
    # monocular SLAM with any EuRoC sequence
    $ ./run_kitti_slam \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -d /path/to/EuRoC/MAV/mav0/ \
        -s ../example/euroc/EuRoC_mono.yaml
    # stereo SLAM with any EuRoC sequence
    $ ./run_kitti_slam \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -d /path/to/EuRoC/MAV/mav0/ \
        -s ../example/euroc/EuRoC_stereo.yaml

The following options are allowed.

.. code-block::

    $ ./run_euroc_slam -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -d, --data-dir arg     directory path which contains dataset
    -s, --setting arg      setting file path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode
    --eval-log             store trajectory and tracking times for evaluation
    --map-db arg           store a map database at this path after SLAM

.. _subsection-example-tum-rgbd:

TUM RGBD dataset
^^^^^^^^^^^^^^^^

Will be written soon.

The following options are allowed.

.. code-block::

    $ ./run_tum_slam -h
    Allowed options:
    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -d, --data-dir arg     directory path which contains dataset
    -a, --assoc arg        association file path
    -s, --setting arg      setting file path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode
    --eval-log             store trajectory and tracking times for evaluation
    --map-db arg           store a map database at this path after SLAM
