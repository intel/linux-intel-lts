.. SPDX-License-Identifier: GPL-2.0

==================================
Kernel driver: intel_tsens_thermal
==================================

Supported chips:
  * Intel Edge.AI Computer Vision platforms: Keem Bay

    Slave address: The address is assigned by the hddl device management
                   driver.

Authors:
    - Thalaiappan, Rathina <rathina.thalaiappan@intel.com>
    - Udhayakumar C <udhayakumar.c@intel.com>

Description
===========
The Intel Edge.AI Computer Vision platforms have memory mapped thermal sensors
which are accessible locally. The intel_tsens_thermal driver handles these
thermal sensor and exposes the temperature to

* the external host similar to the standard SMBUS based thermal sensor
    (like LM73) to the host by registering to the I2C subsystem as
    slave interface (Documentation/i2c/slave-interface.rst).
* the local CPU as a standard thermal device.

In Keem Bay, the four thermal junction temperature points are,
Media Subsystem (mss), NN subsystem (nce), Compute subsystem (cse) and
SOC(Maximum of mss, nce and cse).

Similarity: /drivers/thermal/qcom

Example
=======
Local Thermal Interface:

Temperature reported in Keem Bay on the Linux Thermal sysfs interface.

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

Remote Thermal Interface:

tsens i2c slave driver reports temperature of various subsytem
junction temperature based on table as below.

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
