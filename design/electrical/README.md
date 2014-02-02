"Rockie" Electrical Overview
===================

Batteries
---------
Rockie has 8 li-ion batteries such as [these](https://www.sparkfun.com/products/8483). They are arranged in series, and are voltage regulated seperately (5V for the BeagleBone and Raspberry Pi, 12V for the motors)

Voltage Regulators
-----------------
A 7805 5V voltage regulator like the one seen [here](https://www.sparkfun.com/products/107) is used to provide a constant 5V voltage for the BeagleBone and Raspberry PI.

The 7805 unfortunately is very inefficient for modern day robotics.  The 78XX series has severe energy loss as a lot of the energy it converts is lost as heat.  More modern day voltage regulators exist that have less heat loss and are cheaper than the 7805. - Bryant

Current Source
--------------
[MOSFET](https://www.sparkfun.com/products/10213) will be used to provide a current source for the motors, and Darlington transistors will provide a current source for the sensors, microcontroller and computer.

For combat robots, I've heard good reviews about the Sabertooth Motor Drivers... [Sabertooth](http://www.robotshop.com/en/catalogsearch/result/?q=sabertooth&order=stats_sales_order_count&dir=desc).  These drivers are really good for driving motors between 6 - 30 Volts at up to 60 Amps! - Bryant


