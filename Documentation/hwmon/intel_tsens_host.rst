.. SPDX-License-Identifier: GPL-2.0

==========================
Kernel driver: intel_tsens
==========================

Supported chips:
  * Intel Edge.AI Computer Vision platforms: Keem Bay

    Slave address: The address is assigned by the hddl device management
                   driver.

    Datasheet:
      Documentation/hwmon/intel_tsens_sensor.rst#Remote Thermal Interface

Authors:
    - Thalaiappan, Rathina <rathina.thalaiappan@intel.com
    - Udhayakumar C <udhayakumar.c@intel.com>

Description
===========
The intel_tsens is a temperature sensor driver receiving the junction temperature
from different heating points inside the SOC. The driver will receive the
temperature on SMBUS connection. The reported temperature is in degrees Celsius.

In Keem Bay, the four thermal junction temperature points are,
Media Subsystem (mss), NN subsystem (nce), Compute subsystem (cse) and
SOC(Maximum of mss, nce and cse).

Example
=======
Temperature reported by a Keem Bay on the Linux Thermal sysfs interface.

# cat /sys/class/thermal/thermal_zone*/type
mss
css
nce
soc

# cat /sys/class/thermal/thermal_zone*/temp
0
29210
28478
29210

+-----------+-------------+
| offset    |   Sensor    |
+-----------+-------------+
|   0       |   mss       |
+-----------+-------------+
|   1       |   css       |
+-----------+-------------+
|   2       |   nce       |
+-----------+-------------+
|   3       |   soc       |
+-----------+-------------+

#sudo i2cdetect -l
i2c-8   smbus           SMBus I801 adapter at efa0              SMBus adapte    r

To read mss junction temperature:
#i2cget -y 8 <slave addr> 0x0 w

To read cse junction temperature:
#i2cget -y 8 <slave addr> 0x1 w

To read nce junction temperature:
#i2cget -y 8 <slave addr> 0x2 w

To read overall SoC temperature:
#i2cget -y 8 <slave addr> 0x3 w
