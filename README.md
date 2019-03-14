# ROS driver for Pololu Qik motors controllers

This is a ROS node for the Pololu Qik Motor controllers. It has only been tested on the 2s9v1. It supports multiple Qiks on the same serial bus.

* [Pololu Qik 2s9v1 Dual Serial Motor Controller](https://www.pololu.com/product/1110)

## Parameters:

* **port** -- path to the UART. Default is /dev/ttyACM0

## Subscribers
* **command** -- a Float32MultiArray containing 2 * (number of motor controllerson the serial bus) values. For a single motor controller this would be a Float32MultiArray containing 2 values. Values range from -1.0 to 1.0.

## Publishers
None.

## Services
None.

# Usage notes

