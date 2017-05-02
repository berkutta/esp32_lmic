ESP32 LMIC
================

This software is "just" a port of the popular https://www.research.ibm.com/labs/zurich/ics/lrsc/lmic.html Library to the ESP32 ESP-IDF Environment. It allows to use a RFM95 or SX1276 module to communicate with the LoRaWAN standard. Thus it's possible to e.g. directly communicate with the The Things Network.

It's currently not near stable or something like that. It can send data without any issues with ABP authentication. More is untested and at your own risk.

Usage
=================

You find an example in the "main.c" file. But it's generally just "plain" LMIC.

To make the Application and flash the ESP32 just run the following command:

`make flash monitor`


Environment
=================
I've used the https://github.com/espressif/esp-idf environment.
