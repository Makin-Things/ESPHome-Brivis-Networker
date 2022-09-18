# ESPHome Brivis Networker

## Inspiration

This project was originally inspired by @NZSmartie who did some work on understanding the hardware and decoding parts of the serial protocol. The original project can be found at https://github.com/NZSmartie/bravis-reverse-engineering

## Current status

![ESPHome Brivis Networker PCB](https://raw.githubusercontent.com/Makin-Things/ESPHome-Brivis-Networker/master/Doc/Images/Brivis%20Networker%203D%20PCB.png)

This is currently a work in progress! I have the designed and built a prototype circuit board. The original Brivis Networker used 2 wires to both get power and do serial communications. The intention was to power this circuit in the same way, but unfortunately the amount of current that can be drawn is limited, so while most of the time everything worked, while the ESP32 was transmitting via wifi there would be communication errors on the serial bus. I have since applied a modification to the board to supply the ESP32 and the analog circuitry using an additional external 5V supply. Since doing this the serial communications appears to be rock solid.
I am now slowly working on the ESPHome software to make everything work. At this stage I am able to add the circuit to the serial bus and log all packets that are sent on the serial bus. Since I changed the power supply to the circuit I have seen zero errors!

## Update 19 September 2022

For the past week I have removed my Brivis NC-2 controller and replaced it with my control board. It has been working flawlessly to control the heating in my house using a schedule and a few automations. One point of note is that it is not possible to change the Brivis NC-2 address (ie. it is always the master) so it can't coexist with the new custom controller. My understanding is the from the NC-3 onwards it is possible to change the address.