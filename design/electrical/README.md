"Rockie" Electrical Overview
===================

Batteries
---------
Rockie has 8 li-ion batteries such as [these](https://www.sparkfun.com/products/8483). They are arranged in series, and are voltage regulated seperately (5V for the BeagleBone and Raspberry Pi, 12V for the motors)

Voltage Regulators
-----------------
A 7805 5V voltage regulator like the one seen [here](https://www.sparkfun.com/products/107) is used to provide a constant 5V voltage for the BeagleBone and Raspberry PI.

Current Source
--------------
(MOSFETs)[https://www.sparkfun.com/products/10213] will be used to provide a current source for the motors, and Darlington transistors will provide a current source for the sensors, microcontroller and computer.


