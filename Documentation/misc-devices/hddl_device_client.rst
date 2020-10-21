.. SPDX-License-Identifier: GPL-2.0

=================================
Kernel driver: hddl_device_client
=================================

Supported chips:
  * Intel Edge.AI Computer Vision platforms: Keem Bay

Authors:
    - Thalaiappan, Rathina <rathina.thalaiappan@intel.com>
    - Udhayakumar C <udhayakumar.c@intel.com>


Overview
========

This driver supports hddl device management for Intel Edge.AI Computer Vision
platforms.

This driver supports the following features:

  - Exports deatils of temperature sensor, current sensor and fan controller
    present in Intel Edge.AI Computer Vision platforms to IA host.
  - Enable Time sync of Intel Edge.AI Computer Vision platform with IA host.
  - Handles device connect and disconnect events.
  - Receives slave address from the IA host for memory mapped thermal sensors
    present in SoC (Documentation/hwmon/intel_tsens_sensors.rst).
  - Registers i2c slave device for slaves present in Intel Edge.AI Computer
    Vision platform

Keem Bay platform has
Onchip sensors:

  - Media Subsystem (mss) temperature sensor
  - NN subsystem (nce) temperature sensor
  - Compute subsystem (cse) temperature sensor
  - SOC(Maximum of mss, nce and cse).

Onboard sensors:

  - two lm75 temperature sensors
  - emc2103 fan controller
  - ina3221 current sensor

High-level architecture
=======================
::

        Remote Host IA CPU                          Local Host ARM CPU
        -------------------------------         ----------------------------
        | * Send time as xlink packet |         |* Sync time with IA host  |
        | * receive sensor details    |         |* Prepare and share sensor|
        |   and register as i2c or    |         |  details to IA host as   |
        |   xlink smbus slaves        |         |  xlink packets           |
        -------------------------------         ----------------------------
        |       hddl server           | <=====> |     hddl client          |
        -------------------------------  xlink  ----------------------------

Driver Structure
================

The driver provides a platform device where the ``probe`` and ``remove``
operations are provided.

  - probe: Gets list of external sensors from device-tree entries, identify
    board id and soc id based on configuration from device-tree entries and
    spawn kernel thread to monitor new PCIE devices.

  - init task: Poll for new PCIE device with time interval of 5 seconds and
    creates connect task to setup new device.

  - connect task: Connect task is the main entity which connects to hddl
    device server using xlink and does the basic initialisation and handshaking.
    Additionally it also monitors the hddl device server link down/link up
    events and reinitialise the drivers accordingly in the client side.

  - remove: unregister i2c client devices, i2c adapters and close xlink
    channel.

HDDL Client Sequence – Basic Setup and handshaking with HDDL Device Server
==========================================================================
::

        ,-----.                ,---------.          ,------------.           ,------------------.
        |probe|                |Init task|          |connect task|           |hddl device server|
        `--+--'                `----+----'          `-----+------'           `--------+---------'
           ----.                    |                     |                           |
               | "Parse DT"         |                     |                           |
           <---'                    |                     |                           |
           |                        |                     |                           |
           | ,------------------!.  |                     |                           |
           | |Get sensor details|_\ |                     |                           |
           | |from device tree    | |                     |                           |
           | `--------------------' |                     |                           |
           ----.                    |                     |                           |
               | "Identify Board Id"|                     |                           |
           <---'                    |                     |                           |
           |                        |                     |                           |
           |   "Creates kthread"    |                     |                           |
           |----------------------->|                     |                           |
           |                        |                     |                           |
           | ,-----------------------!.                   |                           |
           | |creates kernel thread  |_\                  |                           |
           | |to check for new device  |                  |                           |
           | `-------------------------'                  |                           |
          ,---------------------!.  ----.                 |                           |
          |check for new device |_\     |                 |                           |
          |with time interval of  | <---'                 |                           |
          |5 seconds              | |                     |                           |
          `-----------------------' |                     |                           |
          ,---------------------!.  |                     |                           |
          |if new device found?.|_\ |                     |                           |
          |creates connect task   | |-------------------->|                           |
          |to setup new device    | |                     |                           |
          `-----------------------' |                     |                           |
           |                       ,-------------------!. |----.                      |
           |                       |setup xlink channel|_\|    |                      |
           |                       |to communicate with  ||<---'                      |
           |                       |server               ||                           |
           |                       `---------------------'|                           |
           |                        |                     |       Get time data       |
           |                        |                     |       from server         |
           |                        |                     | <--------------------------
           |                        |                     |                           |
           |                        |                     |       share board id      |
           |                        |                     | -------------------------->
           |                        |                     |                           |
           |                        |                     |  share total number of    |
           |                        |                     |  sensors available in SoC |
           |                        |                     | -------------------------->
           |                        |                     |                           |
           |                   ,-----------------------!. |                           |
           |                   |For each sensors share |_\|                           |
           |                   |sensor type, name, trip  || -------------------------->
           |                   |temp, trip type          ||                           |
           |                   `-------------------------'|                           |
           |                        |                     |  Receives Send complete.  |
           |                        |                     | <--------------------------
           |                        |                     |                           |
           |                        |                     |----.                      |
           |                        |                     |    | Register xlink i2c   |
           |                        |                     |<---' adapters.            |
           |                        |                     |                           |
           |                        |                     |                           |
           |                        |                     |  Receives slave addr for  |
           |                        |                     |   each salve in SoC       |
           |                        |                     | <--------------------------
           |                        |                     |                           |
           |                        |                     |----.                      |
           |                        |                     |    | Register i2c clients.|
           |                        |                     |<---'                      |
           |                        |                     |                           |
           |                        |                     |----.
           |                        |                     |    | poll for device status
           |                        |                     |<---'
        ,--+--.                ,----+----.          ,-----+------.           ,--------+---------.
        |probe|                |Init task|          |connect task|           |hddl device server|
        `-----'                `---------'          `------------'           `------------------'


XLINK i2c sequence:
===================
::

        ,-----------------.          ,--------.          ,-----.          ,---------.
        |xlink-i2c-adapter|          |I2C core|          |xlink|          |i2c-slave|
        `--------+--------'          `---+----'          `--+--'          `----+----'
                 |                       |                  |                  |
                 |---------------------->|                  |                  |
                 |                       |                  |                  |
                 | ,--------------------------!.            |                  |
                 | |Initialize xlink based i2c|_\           |                  |
                 | |adapters.                   |           |                  |
                 | `----------------------------'           |                  |
                 |                       |                  |                  |
                 |<-----------------------------------------|                  |
                 |                       |                  |                  |
                 | ,--------------------------------!.      |                  |
                 | |I2C request is received as xlink|_\     |                  |
                 | |packet from IA host               |     |                  |
                 | `----------------------------------'     |                  |
                 |                       |                  |                  |
                 |---------------------->|                  |                  |
                 |                       |                  |                  |
                 |                       |  ,---------------------------------!.
                 |                       |  |xlink I2C request is converted to|_\
                 |                       |  |standard i2c request               |
                 |                       |  `-----------------------------------'
                 |                       |                  |                  |
                 |                       | ----------------------------------->|
                 |                       |                  |                  |
                 |                       |  ,----------------------!.          |
                 |                       |  |Linux i2c slave device|_\         |
                 |                       |  |standard request        |         |
                 |                       |  `------------------------'         |
                 |                       |                  |                  |
                 |                       | <-----------------------------------|
                 |                       |                  |                  |
                 |                       |  ,----------------------!.          |
                 |                       |  |Linux i2c slave device|_\         |
                 |                       |  |standard response       |         |
                 |                       |  `------------------------'         |
                 |     I2C response      |                  |                  |
                 |<----------------------|                  |                  |
                 |                       |                  |                  |
                 |                       |                  | ,-------------------------!.
                 |----------------------------------------->| |I2C response is converted|_\
                 |                       |                  | |to xlink packet            |
        ,--------+--------.          ,---+----.          ,--+-`---------------------------'
        |xlink-i2c-adapter|          |I2C core|          |xlink|          |i2c-slave|
        `-----------------'          `--------'          `-----'          `---------'
