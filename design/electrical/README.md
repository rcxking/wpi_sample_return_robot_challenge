"Rockie" Electrical Overview
===================

Batteries
---------
Rockie has 8 li-ion batteries such as [these](https://www.sparkfun.com/products/8483). They are arranged in series, and are voltage regulated seperately (5V for the BeagleBone and Raspberry Pi, 12V for the motors)

Voltage Regulators for Sensors, Computer, and Microcontroller
-----------------
A LM2679 as seen [here](http://www.digikey.com/product-detail/en/LM2679T-5.0%2FNOPB/LM2679T-5.0%2FNOPB-ND/363836) will provide a constant 5V power supply directly from the batteries to the BeagleBone and Raspberry Pi. This will help isolate the motor power from the computer power.

> The 7805 unfortunately is very inefficient for modern day robotics.  The 78XX series has severe energy loss as a lot of the energy it converts is lost as heat.  More modern day voltage regulators exist that have less heat loss and are cheaper than the 7805. - Bryant

> This is very true - I was thinking about using the 7805 just to get a steady 5V, and use a small current from the 7805 to drive a Darlington transistor current source to source the actual power. However, I think you're right, it'll just be easier to just get a more efficient voltage regulator like you suggested. The LM2679 looks pretty good. -Will

Motor Drivers
--------------
A Sabertooth motor-driver found [here](http://www.robotshop.com/en/dimension-engineering-sabertooth-2x25.html) will supply 12V up to 25A for the drive motors, which should be enough.

> For combat robots, I've heard good reviews about the Sabertooth Motor Drivers... [Sabertooth](http://www.robotshop.com/en/catalogsearch/result/?q=sabertooth&order=stats_sales_order_count&dir=desc).  These drivers are really good for driving motors between 6 - 30 Volts at up to 60 Amps! - Bryant

> Great suggestion, it looks like a Sabertooth motor driver [here](http://www.robotshop.com/en/dimension-engineering-sabertooth-2x25.html) would do everything we would need. The $125 price tag stinks, but it will be nice to have overcurrent protection. -Will

