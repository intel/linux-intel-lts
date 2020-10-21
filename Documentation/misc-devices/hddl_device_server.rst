.. SPDX-License-Identifier: GPL-2.0

Kernel driver: hddl_device_server
=================================

Supported chips:
  * Intel Edge.AI Computer Vision platforms: Keem Bay

Authors:
    - Thalaiappan, Rathina <rathina.thalaiappan@intel.com>
    - Udhayakumar C <udhayakumar.c@intel.com>

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

Overview
========

This driver supports hddl device management for Intel Edge.AI Computer Vision
platforms. This driver runs in IA host

This driver supports the following features:

  - Receives deatils of temperature sensor, current sensor and fan controller
    present in Intel Edge.AI Computer Vision platforms.
  - Send Time sync data to Intel Edge.AI Computer Vision platform.
  - Handles device connect and disconnect events.
  - Get free slave address for memory mapped thermal sensors present in SoC
    (Documentation/hwmon/intel_tsens_sensors.rst) and share it with Intel
    Edge.AI Computer Vision platform.
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

Driver Structure
================

The driver provides a platform device where the ``probe`` and ``remove``
operations are provided.

  - probe: spawn kernel thread to monitor new PCIE devices.

  - init task: Poll for new PCIE device with time interval of 5 seconds and
    creates connect task to setup new device.

  - connect task: Connect task is the main entity which connects to hddl
    device client using xlink and does the basic initialisation and handshaking.
    Additionally it also monitors the hddl device client link down/link up
    events and reinitialise the drivers accordingly in the server side.

  - remove: unregister i2c client devices, i2c adapters and close xlink
    channel.

HDDL Server Sequence - Basic Setup and handshaking with HDDL Device Client
==========================================================================
::

          ,-----.            ,---------.          ,------------.           ,------------------.
          |probe|            |Init task|          |connect task|           |hddl device client|
          `--+--'            `----+----'          `-----+------'           `--------+---------'
             ----.                |                     |                           |
                 | "Init char dev"|                     |                           |
             <---'                |                     |                           |
             |                    |                     |                           |
             | ,----------------------!.                |                           |
             | |Initialize char device|_\               |                           |
             | |for ioctls              |               |                           |
             | `------------------------'               |                           |
             | "Creates kthread"  |                     |                           |
             |------------------->|                     |                           |
             |                    |                     |                           |
             | ,-----------------------!.               |                           |
             | |creates kernel thread  |_\              |                           |
             | |to check for new device  |              |                           |
             | `-------------------------'              |                           |
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
             |                   ,-------------------!. |----.                      |
             |                   |setup xlink channel|_\|    |                      |
             |                   |to communicate with  ||<---'                      |
             |                   |client               ||                           |
             |                   `---------------------'|                           |
             |                    |                     |      share time data      |
             |                    |                     |      to client            |
             |                    |                     | -------------------------->
             |                    |                     |                           |
             |                    |                     |     receives board id     |
             |                    |                     | <--------------------------
             |                    |                     |                           |
             |                    |                     |  Gets total number of     |
             |                    |                     |  sensors available in SoC |
             |                    |                     | <--------------------------
             |                    |                     |                           |
             |               ,-----------------------!. |                           |
             |               |For each sensors get   |_\|                           |
             |               |sensor type, name, trip  || <--------------------------
             |               |temp, trip type          ||                           |
             |               `-------------------------'|                           |
             |                    |                     |       Send complete.      |
             |                    |                     | -------------------------->
             |                    |                     |                           |
             |                    |                     |----.                      |
             |                    |                     |    | Register xlink i2c   |
             |                    |                     |<---' adapters.            |
             |                    |                     |                           |
             |                    |                     |                           |
             |                    |                     |    send slave addr for    |
             |                    |                     |     each salve in SoC     |
             |                    |                     | -------------------------->
             |                    |                     |                           |
             |                    |                     |----.                      |
             |                    |                     |    | Register i2c clients.|
             |                    |                     |<---'                      |
             |                    |                     |                           |
             |                    |                     |----.
             |                    |                     |    | poll for device status
             |                    |                     |<---'
          ,--+--.            ,----+----.          ,-----+------.           ,--------+---------.
          |probe|            |Init task|          |connect task|           |hddl device client|
          `-----'            `---------'          `------------'           `------------------'


XLINK i2c sequence:
===================
::

        ,-----------------.          ,--------.          ,---------.          ,-----.
        |xlink-i2c-adapter|          |I2C core|          |i2c-slave|          |xlink|
        `--------+--------'          `---+----'          `----+----'          `--+--'
                 |                       |                    |                  |
                 |---------------------->|                    |                  |
                 |                       |                    |                  |
                 | ,--------------------------!.              |                  |
                 | |Initialize xlink based i2c|_\             |                  |
                 | |adapters.                   |             |                  |
                 | `----------------------------'             |                  |
                 |                       |                    |                  |
                 |                       | <------------------|                  |
                 |                       |                    |                  |
                 |                       |  ,----------------------!.            |
                 |                       |  |Linux i2c slave device|_\           |
                 |                       |  |standard request        |           |
                 |                       |  `------------------------'           |
                 |   i2c request from    |                    |                  |
                 |   clients.            |                    |                  |
                 |<----------------------|                    |                  |
                 |                       |                    |                  |
                 |                       |                    |                  |
                 |-------------------------------------------------------------->|
                 |                       |                    |                  |
                 |                       |  ,----------------------------!.      |
                 |                       |  |I2C request is sent as xlink|_\     |
                 |                       |  |packet to SoC                 |     |
                 |                       |  `------------------------------'     |
                 |                       |                    |                  |
                 |<--------------------------------------------------------------|
                 |                       |                    |                  |
                 |                       |  ,------------------------------!.    |
                 |                       |  |I2C response from SoC as xlink|_\   |
                 |                       |  |packet                          |   |
                 |                       |  `--------------------------------'   |
                 |                       |                    |                  |
                 |---------------------->|                    |                  |
                 |                       |                    |                  |
                 | ,---------------------------!.             |                  |
                 | |xlink response is converted|_\            |                  |
                 | |to standard i2c response.    |            |                  |
                 | `-----------------------------'            |                  |
                 |                       |    i2c response    |                  |
                 |                       | ------------------>|                  |
        ,--------+--------.          ,---+----.          ,----+----.          ,--+--.
        |xlink-i2c-adapter|          |I2C core|          |i2c-slave|          |xlink|
        `-----------------'          `--------'          `---------'          `-----'
