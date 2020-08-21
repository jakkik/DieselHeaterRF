### Simple library for Arduino to control an inexpensive Chinese diesel heater through 433 MHz RF by using a TI CC1101 transceiver.

Replicates the protocol used by the four button "red LCD remote" with an OLED screen, and should probably work if your heater supports this type of remote controller.

You will need an ESP32 and TI CC1101 transceiver.

Connect the SPI bus and GDO2 as follows:

    ESP32         CC1101
    -----         ------
    4   <-------> GDO2
    18  <-------> SCK
    3v3 <-------> VCC
    23  <-------> MOSI
    19  <-------> MISO
    5   <-------> CSn
    GND <-------> GND

### Background information:

There is another project for controlling the heater that seems awesome and very comprehensive, [Afterburner by Ray Jones](http://www.mrjones.id.au/afterburner/). 

But if you’re like me, and you just want a simple way to control the heater the way you want without any expensive parts, or electrical connections to the heater's own control unit, this library might help you.

I wanted a non-invasive way to control the heater, while maintaining the original functionality of the heater and remote controller(s). So I decided to sniff around by tapping into the SPI bus between the remote controller and transceiver chip. Using a logic analyzer, I studied the configuration of the radio and the protocol used between the heater and the remote. 

Required parts can be obtained for less than $ 10 USD. I used an Ebyte E07-M1101S, but there are many different breakout modules that should also work.

Link to E07-M1101S: https://www.aliexpress.com/item/32805699419.html

Happy hacking!
