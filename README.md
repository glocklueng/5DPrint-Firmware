5DPrint Firmware
===================

This is the firmware that powers the Makible printers (currently, the Makibox
A6).  It is based on the Sprinter firmware for RepRap-derivative controllers,
which is itself based on the Tonokips firmware.

Licensed under the GNU GPLv3.
See the LICENSE file or visit http://www.gnu.org/licenses/ for the latest 
version of the license and to find older versions of the GNU GPL licenses.

Copyright (c) 2012-2014 by Makible Limited.
 
This file is part of the 5DPrint Firmware.
 
5DPrint Firmware is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free 
Software Foundation, either version 3 of the License, or (at your option) any 
later version.
 
5DPrint Firmware is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with 
the 5DPrint Firmware.  If not, see <http://www.gnu.org/licenses/>.

5DPrint D8 Build
===================
```
make clean && make
```

Printrboard Rev B Build
===================
```
make clean && make HARDWARE=PRINTRBOARD_REVB
```
