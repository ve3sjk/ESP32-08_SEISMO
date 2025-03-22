This is a work in progress in porting https://github.com/F33RNI/SeismoHome to work with a Freenove ESP32-WROOM Board (2 Pack), Dual-core 32-bit 240 MHz Microcontroller on a Freenove Breakout Board for ESP32 / ESP32-S3 WROVER WROOM

This code contains the esp32 files at this time i still need to upload the webserver files as well.

The additions of the seismo functions have been added to an existing sensor system with MQTT functions, the code is raw. Many funtion toggle defines are used, and load order in setup is touchy, don't move things around between the includes.
