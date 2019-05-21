.. _chapter-trouble-shooting:

================
Trouble Shooting
================


.. _section-trouble-build:

For building
============

#. OpenVSLAM terminates abnormaly soon after **launching** or **optimization with g2o**.

    Please configure and rebuild g2o and OpenVSLAM with ``-DBUILD_WITH_MARCH_NATIVE=OFF`` option for ``cmake``.


.. _section-trouble-slam:

For SLAM
========
