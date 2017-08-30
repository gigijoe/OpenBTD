# OpenBTD - BMW Thermostat Daemon

## Objective

This project is focus on control and monitor thermostat heater of BMW M62TU engine.

To help fuel consumption and part-throttle emissions, the coolant tempature of BMW M62TU engine is about 108°C. It's good for earth but the presure by the higher coolant temperature causes leak of coolant and oil. The thermostat of M62TU engine is built-in with heater in order to open it early by the DME during some conditions. Here I'm trying to control the heater by myself in dorder to lower the coolant temperature and reduce the presure of cooling system.

## Principle

The thermostat starts open at 105°C and full open at 108°C. During some conditions DME will power thermostat heater by grounding it.
1. Engine high load
2. High intake air temperature
3. Coolant temperature reach 113°C
    


    +----------+
    |          |     Thermostat heater
    |   DME    |----------|><|----------12V
    +-----+----+
          |
         GND

## Modification

The BTD is the device that reads coolant temperature from IKE through I-Bus and controls thermostat heater.


                   12V 
                    |
                    | 
                {Resistor} /* Make DME happy, prevent Map Cooling fault code. */
                    |
    +----------+    |
    |          |    |      Thermostat heater
    |   DME    |----+      +----|><|----------12V
    +------+---+           |
           |               |
          GND              |
                           |
    +----------+           |
    |          |           | 
    |   BTD    |-----------+
    +---+---+--+
        |   |
       GND  |                       +----------+
            |                       |          |
    --------+------I-Bus------------|   IKE    |
                                    +----------+

## Hardware

## Software

## Reference

http://www.meeknet.co.uk/e31/BMW_M60_M62_M62TU_Engine_Cooling_System.htm

http://www.meeknet.co.uk/e31/M62B44_Engine.pdf

