# Balance-Bot

A Balance Bot is a two wheel robot,  Atmenga 328 (Arduino Nanao) is the microcontroller used. It uses a PID control architecture implemented to control the position, velocity, rotation. 

The tilt angle (pitch) is obtained using MPU6050 sensor. IMU sensors usually consist of two or more parts. Listing them by priority, they are the accelerometer, gyroscope, magnetometer, and altimeter. The MPU 6050 is a 6 DOF (degrees of freedom) or a six-axis IMU sensor, which means that it gives six values as output: three values from the accelerometer and three from the gyroscope. The MPU 6050 is a sensor based on MEMS (micro electro mechanical systems) technology. Both the accelerometer and the gyroscope are embedded inside a single chip. This chip uses I2C (inter-integrated circuit) protocol for communication.

DMP stands for Digital Motion Processing. The MPU 6050 has a built-in motion processor. It processes the values from the accelerometer and gyroscope to give us accurate 3D values.

Also, you will need to wait about 10 seconds before you get accurate values from the Arduino MPU 6050, after which the values will begin to stabilize. Just check out the video below to see if yours is working correctly.

The Working of Balance Bot has been demonstrated in the Video Demo folder.
