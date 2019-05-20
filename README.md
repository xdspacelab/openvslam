# OpenVSLAM
[![Documentation Status](https://readthedocs.org/projects/openvslam/badge/?version=master)](https://openvslam.readthedocs.io/en/master/?badge=master)

## Overview

### Features

To be written.

**We provide a [document](https://openvslam.readthedocs.io/en/latest/index.html) for installation and tutorial.**

### License

**2-clause BSD license** (see [LICENSE](./LICENSE))

The following files are third-party libraries.

- `./3rd/json/include/nlohmann/json.hpp` : part of [nlohmann/json \[v3.6.1\]](https://github.com/nlohmann/json) (MIT license)
- `./3rd/popl` : part of [badaix/popl \[v1.2.0\]](https://github.com/badaix/popl) (MIT license)
- `./3rd/spdlog` : part of [gabime/spdlog \[v1.3.1\]](https://github.com/gabime/spdlog) (MIT license)
- `./src/openvslam/solver/pnp_solver.cc` : part of [laurentkneip/opengv](https://github.com/laurentkneip/opengv) (3-clause BSD license)

### Contributors

- Shinya Sumikura ([@shinsumicco](https://github.com/shinsumicco))
- Mikiya Shibuya ([@MikiyaShibuya](https://github.com/MikiyaShibuya))
- Ken Sakurada ([@kensakurada](https://github.com/kensakurada))

## Installation

Please see [**Installation**](https://openvslam.readthedocs.io/en/latest/installation.html) chapter in the [document](https://openvslam.readthedocs.io/en/latest/index.html).

## Tutorial

Please see [**Simple Tutorial**](https://openvslam.readthedocs.io/en/latest/simple_tutorial.html) chapter in the [document](https://openvslam.readthedocs.io/en/latest/index.html).

If you would like to run visual SLAM with standard benchmarking datasets (e.g. KITTI Odometry dataset), please see [**SLAM with standard datasets**](https://openvslam.readthedocs.io/en/latest/example.html#slam-with-standard-datasets) section in the [document](https://openvslam.readthedocs.io/en/latest/index.html).

## Working on

- ROS support
- IMU integration
- Implementation of extra camera models
- Refactoring
