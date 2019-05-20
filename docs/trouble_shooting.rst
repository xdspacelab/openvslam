.. _chapter-trouble-shooting:

================
Trouble Shooting
================


.. _section-trouble-build:

For building
============

#. OpenVSLAM terminates abnormaly soon after **launching** or **optimization with g2o**.

    Please try to configure g2o and OpenVSLAM with ``-DBUILD_WITH_MARCH_NATIVE=OFF`` option when executing ``cmake``.


.. _section-trouble-slam:

For SLAM
========
