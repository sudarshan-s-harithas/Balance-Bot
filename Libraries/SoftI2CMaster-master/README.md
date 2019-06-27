SoftI2CMaster
=============

Software I2C Arduino library

This is a very fast and light-weight software I2C-master library
written in AVR assembler, which is based on Peter Fleury's software
I2C library http://homepage.hispeed.ch/peterfleury/avr-software.html
. It can use almost any pins on any AVR chip to drive the SDA and SCL lines.

It assumes a single master and does not support bus arbitration. It
allows for clock stretching by slave devices and also can detect lock
ups of the I2C bus, i.e., if the SCL line is held low infinitely.

Even on 1MHz systems, you can get a transfer speed of around 40 kbit/sec, so
you can use it to interface with SMbus devices.

In the program text before including this library, you have to define
all the necessary constants such as  SDA_PIN, SDA_PORT, SCL_PIN, and
SCL_PORT.

For a full description see: http://playground.arduino.cc/Main/SoftwareI2CLibrary

Note: The port ports H and above on ATmega256 are not supported. And,
since it makes heavy use of assembler code, it does not run on ARM
MCUs (Due, Zero, etc.).
