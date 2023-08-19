# TWC Controller

This is an [esphome](https://esphome.io) external component of the Tesla Wall Connector Gen2 controller which was previously a standalone Arduino application (https://github.com/jnicolson/twc-controller).  The change to esphome means no more worrying about the boilerplate type setup of Wifi, Over the air updates, MQTT connections, etc.  Instead the focus just becomes on the Tesla load sharing protocol.

Credit goes to [WinterDragoness](https://teslamotorsclub.com/tmc/members/winterdragoness.40930/) from the Tesla Motor Club forums for figuring out the protocol (with many others in the forum also giving help) and [Craig 128](https://teslamotorsclub.com/tmc/members/craig-128.113283/) for the original C code I used as a reference.  Both their projects are linked in the links section below - without both of them this wouldn't have been possible.

## Features

This is designed to work with Home Assistant by exposing a single controllable number - the current to make available to the charger.  This can be automated in Home Assistant based on whatever data is available in Home Assistant (i.e. time of day, solar availability).

The charger also exposes many sensors for Voltage, Current, kWh provided over the lifetime, serial number, connected VIN.  These are all visible in the example YAML file twc.yaml.

## Hardware

This should run on any ESP32 based board with an RS485 transceiver which supports reading and writing.  This code is tested on a custom board with hardware design available at https://github.com/jnicolson/TWCController

## Installation

Included in this repository is an example esphome yaml file. This assumes:
* Secondary UART connected to GPIO5 and GPIO19 (update as required)
* RS485 Receive Enable connected to GPIO18

## Other Projects

* [TWC](https://github.com/craigpeacock/TWC)
* [TWCManager](https://github.com/ngardiner/TWCManager)
* [TWCManager (original)](https://github.com/dracoventions/TWCManager)
