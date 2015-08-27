# hexa_control

This package is to be used in conjuction with an optimisation technique that will provide the parameters used to generate the motion of each limb of an hexapod. More precisely, we use this with the Intelligent Trial and Error algorithm.

Please note that this is a work in progress and it might not work as well as expected.

## Dependencies
hexa_control needs the nav_msgs package that can be retreived by APT-get with 'sudo apt-get install ros-YourDistro-nav-msgs.

In addition, it requires libdynamixel to be available on the system and manually added to CMakeLists.txt.
