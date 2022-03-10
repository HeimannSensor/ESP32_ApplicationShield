# ApplicationShield
Sample code for the ESP32 DevkitC 32D and HTPAd Application Shield (written in the Arduino IDE).
  
In this repository you will find programs for all supported HTPAd sensors.
Please choose the correct program folder for your sensor. You have to replace the dummy def.h file in the folder with the matching one from the all_sensor_defs folder.
Otherwise the program will use a wrong lookup table and you are not able to observe correct pixel temperatures. 


Info to the PCB "ESP32 Application Shield"
-------------------------------------------
This Application Shield can be used with an ESP32 development board. It is designed to facilitate the access to our thermopile arrays for the fast way to get the thermal image from the sensor. The full code is provided and completely open source. It includes all required steps from reading the EEPROM to the calculation of the thermal image. The C++ code can be viewed and modified via the Arduino IDE. The PCB is designed as an ESP 32 DevkitC 32 D extension.

Supported sensor types:

       TO46     TO39
I2C
SPI
