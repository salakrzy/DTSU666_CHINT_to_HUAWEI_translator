# DTSU666_CHINT_to_HUAWEI_translator
This project uses ESP32 to translate MODBUS messages of CHINT's DTSU666 Energy Meter into HUAWEI DTSU666H format.
This project translates the DTSU666 CHINT Power Meter registers addreses to Huawei DTSU666H Power Meter addreses.

All data received from CHINT counter could be send to MQTT Broker.

Project base on the eMODBUS librrary  https://github.com/eModbus/eModbus 

Information on how to prepare enviroment , compile and flash ESP32 modules can be found at
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#

Project uses the Arduino.h librrary. How to instal this librrary in espressif enviroment,  you can read at
https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html

Description how to assembling the hardware you will found in the /doc project directory . 

After instalation Espressif ESP-IDF enviroment for ESP32 , you can execute commands to compiling and flash this project. Go to the directory where you copied project files and execute this commands

idf.py set-target esp32

idf.py menuconfig // not nessesery if you want to use configuration prepared by me.

idf.py -p com8 build flash monitor // change number COM port depending your configuration.

After this commands build directory will be created and the ESP32 module flashed .

In used RS485 interface the R5 and R6 resistors are 20k ohm it is to much for MOdbus standard, if you have transmission errors, change this resistors to 2k ohm.
If you change the R5 and R6 resistors,  you don't need remove the R7 120 ohm resistor.

Good Luck:-)
