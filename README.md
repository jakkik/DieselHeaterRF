### Simple Arduino library for controlling and monitoring inexpensive, unbranded Chinese diesel heaters through 433 MHz RF by using a TI CC1101 transceiver.

Library replicates the protocol used by the smaller four button remote with an OLED screen (see below), and should probably work if your heater supports this type of remote controller.

![Red remote](https://github.com/jakkik/DieselHeaterRF/blob/master/doc/red-remote.jpg?raw=true "Controllers with this type of remote are supported")

### Parts needed

* ESP32
* TI CC1101 transceiver module

Connect the SPI bus and GDO2 as follows:

![Wiring diagram](https://github.com/jakkik/DieselHeaterRF/blob/master/doc/heateresp_bb.jpg?raw=true "Wiring diagram")

    ESP32         CC1101
    -----         ------
    4   <-------> GDO2
    18  <-------> SCK
    3v3 <-------> VCC
    23  <-------> MOSI
    19  <-------> MISO
    5   <-------> CSn
    GND <-------> GND

### Features

All features of the physical remote are available through the library.

#### Get current state of the heater, including:
* Heater power state
* Current temperature setpoint
* Current pump frequency setpoint
* Ambient temperature
* Heat exchanger temperature
* Operating mode (thermostat or fixed pump frequency)
* Power supply voltage
* Current state (glowing, heating, cooling...)
* RSSI of the received signal

#### Commands
* Power on / off
* Temperature setpoint up / down (when in "auto", thermostat mode)
* Pump frequency up / down (when in "manual", fixed pump freq. mode)
* Operating mode auto / manual

#### Pairing mode
* Find the heater address

### Background information:

There is another project for a stand-alone device for controlling the heater that seems awesome and very comprehensive, [Afterburner by Ray Jones](http://www.mrjones.id.au/afterburner/). 

But if you’re like me, and you just want a simple way to control or monitor the heater without any expensive parts or electrical connections to the heater's own control unit, and don't mind rolling out your own software, this library might help you.

I wanted a non-invasive way to control the heater, while maintaining the original functionality of the heater and remote controller(s). So I decided to sniff around by tapping into the SPI bus between the remote controller's MCU and the transceiver chip. Using a logic analyzer and the CC1101 datasheet, I studied the configuration of the radio and the protocol used between the heater and the remote. 

Required parts can be obtained for less than $ 10 USD. I used an Ebyte E07-M1101S, but there are many different breakout modules that should also work.

Link to E07-M1101S: https://www.aliexpress.com/item/32805699419.html
(Note: this module has teeny-weeny PCB holes and dense 1.27 mm pitch, so if you want a more easily solderable breadboard-friendly 2.54 mm (.1 in) pitch version, you might want to choose a different module.)

### Disclaimer

Feel free to use this library as you please, but do it at your own risk!

Happy hacking! 🤓
