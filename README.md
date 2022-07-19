# ESPHome Brivis Networker

## Inspiration

This project was originally inspired by @NZSmartie who did some work on understanding the hardware and decoding parts of the serial protocol. The original project can be found at https://github.com/NZSmartie/bravis-reverse-engineering

## Current status

This is currently a work in progress! I have the designed and built a prototype circuit board. The original Brivis Networker used 2 wires to both get power and do serial communications. The intention was to power this circuit in the same way, but unfortunately the amount of current that can be drawn is limited, so while most of the time everything worked, while the ESP32 was trasnmitting via wifi there would be communication errors on the serial bus. I have since applied a modification to the board to supply the ESP32 and the analog circuitry using an additional external 5V supply. Since doing this the serial communications appears to be rock solid.
I am now slowly working on the ESPHome software to make everything work. At this stage I am able to add the circuit to the serial bus and log all packets that are sent on the serial bus. Since I changed the power supply to the circuit I have seen zero errors!



