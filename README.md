# hexa_control

This package is to be used in conjuction with an optimisation technique that will provide the parameters used to generate the motion of each limb of an hexapod. More precisely, we use this with the Intelligent Trial and Error algorithm.

Please note that this is a work in progress and it might not work as well as expected.

## Authors
Original author : Antoine Cully

## Dependencies
hexa_control needs the nav_msgs and geometry_msgs packages that can be retreived by APT-get with `sudo apt-get install ros-YourDistro-nav-msgs` and `sudo apt-get install ros-YourDistro-geometry-msgs`.

In addition, it requires libdynamixel to be available on the system. It can be found on the **HOME** folder by default. You can change where to search for libdynamixel by setting the environmental variable *RESIBOTS_DIR* to the path where libdynamixel is located.

## Odometry
This package needs some odometry information to estimate the performance of the trial. To get it, the program subscribes to the "vo" topic.

## Service Transfert for giving oscillator parameters
hexapod_server advertises a service called Transfert with which a consumer can send parameters to be used by the oscillators.
