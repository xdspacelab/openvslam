# OpenVSLAM
[![Documentation Status](https://readthedocs.org/projects/openvslam/badge/?version=master)](https://openvslam.readthedocs.io/en/master/?badge=master)

## Overview

<img src="./docs/img/teaser.png">

### Features

To be written.

**We provide a [document](https://openvslam.readthedocs.io/) for installation and tutorial.**

### License

**2-clause BSD license** (see [LICENSE](./LICENSE))

The following files are derived from third-party libraries.

- `./3rd/json` : part of [nlohmann/json \[v3.6.1\]](https://github.com/nlohmann/json) (MIT license)
- `./3rd/popl` : part of [badaix/popl \[v1.2.0\]](https://github.com/badaix/popl) (MIT license)
- `./3rd/spdlog` : part of [gabime/spdlog \[v1.3.1\]](https://github.com/gabime/spdlog) (MIT license)
- `./src/openvslam/solver/pnp_solver.cc` : part of [laurentkneip/opengv](https://github.com/laurentkneip/opengv) (3-clause BSD license)
- `./src/openvslam/feature/orb_point_pairs.h` : part of [opencv/opencv](https://github.com/opencv/opencv) (3-clause BSD License)

### Contributors

- Shinya Sumikura ([@shinsumicco](https://github.com/shinsumicco))
- Mikiya Shibuya ([@MikiyaShibuya](https://github.com/MikiyaShibuya))
- Ken Sakurada ([@kensakurada](https://github.com/kensakurada))

## Installation

Please see [**Installation**](https://openvslam.readthedocs.io/en/master/installation.html) chapter in the [document](https://openvslam.readthedocs.io/).

## Tutorial

Please see [**Simple Tutorial**](https://openvslam.readthedocs.io/en/master/simple_tutorial.html) chapter in the [document](https://openvslam.readthedocs.io/).

If you would like to run visual SLAM with standard benchmarking datasets (e.g. KITTI Odometry dataset), please see [**SLAM with standard datasets**](https://openvslam.readthedocs.io/en/master/example.html#slam-with-standard-datasets) section in the [document](https://openvslam.readthedocs.io/).

## Working on

- ROS support
- IMU integration
- Implementation of extra camera models
- Refactoring
