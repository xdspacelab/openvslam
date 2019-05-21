.. _chapter-simple-tutorial:

===============
Simple Tutorial
===============

TL; DR
^^^^^^

You can check the normal operation of OpenVSLAM with the following commands.
Please copy and paste to your terminal.

.. code-block:: bash

    # at the build directory of openvslam ...
    $ pwd
    /path/to/openvslam/build/
    $ ls
    run_euroc_slam   lib/   ...

    # download a sample dataset from Google Drive
    FILE_ID="1mYJ_W6WsMjOqoNGaEoOvIe17NuEMYcSz"
    curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_entrance_hall_1.zip
    unzip aist_entrance_hall_1.zip

    # download an ORB vocabulary from Google Drive
    FILE_ID="1wUPb328th8bUqhOk-i8xllt5mgRW4n84"
    curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o orb_vocab.zip
    unzip orb_vocab.zip

    # run tracking and mapping
    ./run_movie_slam -v ./orb_vocab/orb_vocab.dbow2 -m ./aist_entrance_hall_1/movie.mp4 -s ./aist_entrance_hall_1/config.yaml --frame-skip 3 --map-db map.msg

    # run localization
    ./run_movie_localization -v ./orb_vocab/orb_vocab.dbow2 -m ./aist_entrance_hall_1/movie.mp4 -s ./aist_entrance_hall_1/config.yaml --frame-skip 3 --map-db map.msg


Sample Datasets
^^^^^^^^^^^^^^^

You can experience OpenVSLAM processing with various types of movies in this section.
If you want to run OpenVSLAM with standard benchmarking detasets, please see :ref:`this section <section-example-standard-datasets>`.

Start by downloading some datasets you like.


.. list-table::
    :header-rows: 1
    :widths: 8, 8, 2, 8

    * - name
      - camera model
      - length
      - download link
    * - aist_entrance_hall_1
      - equirectangular (mono)
      - 0:54
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_entrance_hall_2
      - equirectangular (mono)
      - 0:54
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_factory_A_1
      - equirectangular (mono)
      - 1:55
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_factory_A_2
      - equirectangular (mono)
      - 1:54
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_factory_B_1
      - equirectangular (mono)
      - 1:04
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_factory_B_2
      - equirectangular (mono)
      - 1:34
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_living_lab_1
      - equirectangular (mono)
      - 2:16
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_living_lab_2
      - equirectangular (mono)
      - 1:47
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_living_lab_3
      - equirectangular (mono)
      - 2:06
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_stairs_A_1
      - equirectangular (mono)
      - 2:27
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_stairs_B_1
      - equirectangular (mono)
      - 2:55
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_store_1
      - equirectangular (mono)
      - 1:12
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_store_2
      - equirectangular (mono)
      - 1:44
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__
    * - aist_store_3
      - equirectangular (mono)
      - 1:18
      - `link <https://drive.google.com/open?id=1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4>`__


After downloading and uncompressing a zip file, you will find a movie and a config file under an uncompressed directory.


.. code-block:: bash

    $ ls dataset_name_X/
    config.yaml  movie.mp4


Please put the dataset at the directory you like.

| Additionally, please download a vocabulary file for DBoW2 from `here <https://drive.google.com/open?id=1wUPb328th8bUqhOk-i8xllt5mgRW4n84>`__.
| After uncompressing it, you can find ``orb_vocab.dbow2`` in an uncompressed directory.

In the following, we use ``aist_living_lab_1`` and ``aist_living_lab_2`` datasets as an example.


Tracking and Mapping
^^^^^^^^^^^^^^^^^^^^

Let's try to run SLAM and create a map database file with ``aist_living_lab_1`` dataset.
You can use ``./run_movie_slam`` for SLAM processing with a movie.


.. code-block::

    # at the build directory of OpenVSLAM
    $ ls
    ...
    run_movie_slam
    ...
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


Please execute the following command to start a SLAM processing.
The paths should be changed according to your environment.


.. code-block::

    $ ./run_movie_slam \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -s /path/to/aist_living_lab_1/config.yaml \
        -m /path/to/aist_living_lab_1/movie.mp4 \
        --frame-skip 3 \
        --map-db aist_living_lab_1_map.msg


Then, you can watch a frame viewer and map viewer.
If the two viewers are not launched at all, please check whether any paths are wrong or not.


.. NOTE ::

    If OpenVSLAM terminates abnormaly soon after initialization, rebuild g2o and OpenVSLAM with ``-DBUILD_WITH_MARCH_NATIVE=OFF`` option for ``cmake`` configulation.


.. image:: ./img/slam_frame_viewer_1.png
    :width: 640px
    :align: center


.. image:: ./img/slam_map_viewer_1.png
    :width: 640px
    :align: center


.. code-block:: none

    [2019-05-20 17:52:41.677] [I] config file loaded: /path/to/aist_living_lab_1/aist_living_lab_1/config.yaml
      ___               __   _____ _      _   __  __  
     / _ \ _ __  ___ _ _\ \ / / __| |    /_\ |  \/  | 
    | (_) | '_ \/ -_) ' \\ V /\__ \ |__ / _ \| |\/| | 
     \___/| .__/\___|_||_|\_/ |___/____/_/ \_\_|  |_| 
          |_|                                         

    Copyright (C) 2019,
    National Institute of Advanced Industrial Science and Technology (AIST)
    All rights reserved.

    This is free software,
    and you are welcome to redistribute it under certain conditions.
    See the LICENSE file.

    Camera Configuration:
    - name: RICOH THETA S 960
    - setup: Monocular
    - fps: 30
    - cols: 1920
    - rows: 960
    - color: RGB
    - model: Equirectangular
    ORB Configuration:
    - number of keypoints: 2000
    - scale factor: 1.2
    - number of levels: 8
    - initial fast threshold: 20
    - minimum fast threshold: 7
    - edge threshold: 19
    - patch size: 31
    - half patch size: 15
    - mask rectangles:
      - [0, 1, 0, 0.1]
      - [0, 1, 0.84, 1]
      - [0, 0.2, 0.7, 1]
      - [0.8, 1, 0.7, 1]
    Tracking Configuration:

    [2019-05-20 17:52:41.678] [I] loading ORB vocabulary: /path/to/orb_vocab/orb_vocab.dbow2
    [2019-05-20 17:52:42.037] [I] startup SLAM system
    [2019-05-20 17:52:42.038] [I] start local mapper
    [2019-05-20 17:52:42.038] [I] start loop closer
    [2019-05-20 17:52:42.395] [I] initialization succeeded with E
    [2019-05-20 17:52:42.424] [I] new map created with 191 points: frame 0 - frame 2
    [2019-05-20 17:53:39.092] [I] detect loop: keyframe 36 - keyframe 139
    [2019-05-20 17:53:39.094] [I] pause local mapper
    [2019-05-20 17:53:39.303] [I] resume local mapper
    [2019-05-20 17:53:39.303] [I] start loop bundle adjustment
    [2019-05-20 17:53:40.186] [I] finish loop bundle adjustment
    [2019-05-20 17:53:40.186] [I] updating map with pose propagation
    [2019-05-20 17:53:40.194] [I] pause local mapper
    [2019-05-20 17:53:40.199] [I] resume local mapper
    [2019-05-20 17:53:40.199] [I] updated map
    [2019-05-20 17:55:36.218] [I] shutdown SLAM system
    [2019-05-20 17:55:36.218] [I] encoding 1 camera(s) to store
    [2019-05-20 17:55:36.218] [I] encoding 301 keyframes to store
    [2019-05-20 17:55:37.906] [I] encoding 19900 landmarks to store
    [2019-05-20 17:55:38.819] [I] save the MessagePack file of database to aist_living_lab_1_map.msg
    median tracking time: 0.045391[s]
    mean tracking time: 0.0472221[s]
    [2019-05-20 17:55:40.087] [I] clear BoW database
    [2019-05-20 17:55:40.284] [I] clear map database


After terminating, you can find a map database file ``aist_living_lab_1_map.msg``.


.. code-block::

    $ ls
    ...
    aist_living_lab_1_map.msg
    ...


The format of map database files is `MessagePack <https://msgpack.org/>`_, so you can reuse created maps for any third-party applications as well as for OpenVSLAM.


Localization
^^^^^^^^^^^^

Next, we try to localize frames in ``aist_living_lab_2`` dataset using the created map file ``aist_living_lab_1_map.msg``.
You can use ``./run_movie_localization`` for localization processing with a movie.


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


Please execute the following command to start a localization processing.
The paths should be changed according to your environment.


.. code-block::

    $ ./run_movie_localization \
        -v /path/to/orb_vocab/orb_vocab.dbow2 \
        -s /path/to/aist_living_lab_2/config.yaml \
        -m /path/to/aist_living_lab_2/movie.mp4 \
        --frame-skip 3 \
        --map-db aist_living_lab_1_map.msg


Then, you can watch a frame viewer and map viewer.
If the two viewers are not launched at all, please check whether any paths are wrong or not.


You can find the current frame is localized based on the prebuild map.


.. image:: ./img/localize_frame_viewer_1.png
    :width: 640px
    :align: center


.. code-block:: none

    [2019-05-20 17:58:54.728] [I] config file loaded: /path/to/aist_living_lab_2/config.yaml
      ___               __   _____ _      _   __  __  
     / _ \ _ __  ___ _ _\ \ / / __| |    /_\ |  \/  | 
    | (_) | '_ \/ -_) ' \\ V /\__ \ |__ / _ \| |\/| | 
     \___/| .__/\___|_||_|\_/ |___/____/_/ \_\_|  |_| 
          |_|                                         

    Copyright (C) 2019,
    National Institute of Advanced Industrial Science and Technology (AIST)
    All rights reserved.

    This is free software,
    and you are welcome to redistribute it under certain conditions.
    See the LICENSE file.

    Camera Configuration:
    - name: RICOH THETA S 960
    - setup: Monocular
    - fps: 30
    - cols: 1920
    - rows: 960
    - color: RGB
    - model: Equirectangular
    ORB Configuration:
    - number of keypoints: 2000
    - scale factor: 1.2
    - number of levels: 8
    - initial fast threshold: 20
    - minimum fast threshold: 7
    - edge threshold: 19
    - patch size: 31
    - half patch size: 15
    - mask rectangles:
      - [0, 1, 0, 0.1]
      - [0, 1, 0.84, 1]
      - [0, 0.2, 0.7, 1]
      - [0.8, 1, 0.7, 1]
    Tracking Configuration:

    [2019-05-20 17:58:54.729] [I] loading ORB vocabulary: /path/to/orb_vocab/orb_vocab.dbow2
    [2019-05-20 17:58:55.083] [I] clear map database
    [2019-05-20 17:58:55.083] [I] clear BoW database
    [2019-05-20 17:58:55.083] [I] load the MessagePack file of database from aist_living_lab_1_map.msg
    [2019-05-20 17:58:57.832] [I] decoding 1 camera(s) to load
    [2019-05-20 17:58:57.832] [I] load the tracking camera "RICOH THETA S 960" from JSON
    [2019-05-20 17:58:58.204] [I] decoding 301 keyframes to load
    [2019-05-20 17:59:02.013] [I] decoding 19900 landmarks to load
    [2019-05-20 17:59:02.036] [I] registering essential graph
    [2019-05-20 17:59:02.564] [I] registering keyframe-landmark association
    [2019-05-20 17:59:03.161] [I] updating covisibility graph
    [2019-05-20 17:59:03.341] [I] updating landmark geometry
    [2019-05-20 17:59:04.189] [I] startup SLAM system
    [2019-05-20 17:59:04.190] [I] start local mapper
    [2019-05-20 17:59:04.191] [I] start loop closer
    [2019-05-20 17:59:04.195] [I] pause local mapper
    [2019-05-20 17:59:04.424] [I] relocalization succeeded
    [2019-05-20 18:01:12.387] [I] shutdown SLAM system
    median tracking time: 0.0370831[s]
    mean tracking time: 0.0384683[s]
    [2019-05-20 18:01:12.390] [I] clear BoW database
    [2019-05-20 18:01:12.574] [I] clear map database


If you set ``--mapping`` option, the mapping module is enabled and the map will be extended.
