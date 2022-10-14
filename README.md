# INA219_test_220821-213658-esp32dev

Test program for running two ina219 voltage/current monitors from an ESP32.

There are two sensors:

sensor1 is for the house battery. This is typically a few tens of amps max so I use a 30A 75mV shunt.

Sensor2 is for the engine starting battery. I'm not interested in the current so I only use the voltage sensor.

