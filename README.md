# DTSU666_CHINT_to_HUAWEI_translator
Project was modyfied to use on Platformio with new eModbus librrary version. 

This project uses ESP32 to translate MODBUS messages from CHINT's DTSU666 Energy Meter into HUAWEI DTSU666H format.
This project translates the DTSU666 CHINT Power Meter registers addreses to Huawei DTSU666H Power Meter addreses.

All received data could be send  parallel over WiFi to MQTT server .

Project base on the eMODBUS librrary  https://github.com/eModbus/eModbus and native for platformio ESP32 libraries

Description how to assembling the hardware you will found in the /doc project directory . In used RS485 interface the R5 and R6 resistors are 20k ohm it is to much for MOdbus standard, if you have transmission errors, change this resistors to 2k ohm.
If you change the R5 and R6 resistors,  you don' need remowe the R7 120 ohm resistor.

You should change your private credentials at this place in main.cpp file

const char* ssid = "wifi username";

const char* password = "wifi password";

const char* mqttServer = "mqtt server";

const int mqttPort = 1883;

const char* mqttUser = "mqtt username";

const char* mqttPassword = "mqtt password";



Good Luck:-)
