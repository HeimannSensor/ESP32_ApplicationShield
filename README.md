Hint: You will find the installation and handling guide in the user manual: [ApplicationShieldESP23_user_manual.pdf](https://github.com/HeimannSensor/ESP32_ApplicationShield/files/8231114/ApplicationShieldESP23_user_manual.pdf)

![Logo-farbe-klein-ppt-farbe](https://user-images.githubusercontent.com/59830049/157676024-adb2230d-1a19-42d3-8bc2-cc392b60e4d7.svg)

# Repository Info
This repository contains the sample code for the ESP32 DevkitC 32E and HTPAd Application Shield (written in the Arduino IDE).
  
In this repository you will find programs for all supported HTPAd sensors.
Please choose the correct program folder for your sensor. You have to replace the dummy def.h file in the folder with the matching one from the all_sensor_defs folder.
Otherwise the program will use a wrong lookup table and you are not able to observe correct pixel temperatures. 


Info to the PCB "ESP32 Application Shield"
-------------------------------------------
This Application Shield can be used with an ESP32 development board. It is designed to facilitate the access to our thermopile arrays for the fast way to get the thermal image from the sensor. The full code is provided and completely open source. It includes all required steps from reading the EEPROM/Flash to the calculation of the thermal image. The C++ code can be viewed and modified via the Arduino IDE. The PCB is designed as an ESP 32 DevkitC 32 E extension. The source code includes three ways to interact with the sensor:
- via WIFI you can stream thermal images in our GUI
- via the serial monitor you can observe the sensor data as text output
- as Access Point the ESP32 generates its own wifi network and allows your PC to connect with it

All modes contain in the same code and you can activate by activating the matching define

Serial
-------------------------------------------
This mode prints all results in the serial monitor of the Arduino IDE. Here the EEPROM/Flash content and sensor voltages can be visualized. It’s easy to interact with the sensor by sending the characters depending on the menu function you want.
Benefits:
- shows EEPROM/Flash content in hexadecimal or associated data type ( short, long, ...)
- prints results after each calculation step
- understand the way from raw pixel voltages to the final compensated thermal image

WIFI | AccessPoint
-------------------------------------------
Via WIFI you can connect your thermopile with the Heimann Sensor GUI to stream continuously. With the GUI streaming the sensor images in temperature or voltage mode is possible. Also, you can change user settings, like clock, ADC resolution and emissivity factor.
Benefits:
- false color visualization of images
- stream continuously
-  switch between temperature and voltage mode
-  record/replay
-  change user settings
