## 5DPrint Firmware

5DPrint Firmware is the primary firmware that runs on the Makibox 3D printer, which uses the 5DPrint  D8 Controller Board and the Printrboard Rev B. It is mainly derived from Sprinter and has a GPL v3 License. 

## Features

Movement

* Look ahead 
* Arc support
* Interrupt-based movement with real linear acceleration
* On-the-fly microstep resolution support
* Current limit support on stepper motor drivers

Heating

* Support 1 hot end and 1 hot bed
* Heating Power reporting
* Heating Power limitation on hot bed
* Interrupt-based temperature protection

Misc

* SD Card support
* SD Card Autoprint support
* EEPROM storage of settiings
* Fan support

For more information, please refer to the [Wiki page](https://bitbucket.org/makible/5dprint-firmware/wiki/Home).

## Compile notes

5DPrint D8 Controller Board Build
===================
```
make clean && make
```

Printrboard Rev B Build
===================
```
make clean && make HARDWARE=PRINTRBOARD_REVB
```
