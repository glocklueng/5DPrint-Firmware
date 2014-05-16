## 5DPrint Firmware

5DPrint Firmware is the primary firmware that runs on the Makibox 3D printer, which uses the 5DPrint  D8 Controller Board and the Printrboard Rev B. It is mainly derived from Sprinter and has a GPL v3 License. 

## Features

Movement

* Look ahead 
* Arc support (G2-G3)
* Interrupt-based movement with real linear acceleration
* On-the-fly microstep resolution support (M907)
* Current limit support on stepper motor drivers (M906)

Heating

* Support 1 hot end and 1 hot bed
* Periodic temperature report (M203)
* Heating Power reporting
* Two stage heating power limitation on hot bed (M305)
* Interrupt-based temperature protection 
* PID Temperature control
* PID Autotune (M303)

Misc

* SD Card support
* SD Card Autoprint support (M31)
* EEPROM storage of settiings (M500-M503)
* Fan support
* GPIO pin support for 5DPrint D8 Controller Board (M908)
* Buzzer support (M300)
* Software jump to bootloader mode (M852)
* USB communication LED

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
