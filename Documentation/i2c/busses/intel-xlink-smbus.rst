.. SPDX-License-Identifier: GPL-2.0

==========================
Kernel driver: xlink_smbus
==========================

Supported chips:
  * Intel Edge.AI Computer Vision platforms: Keem Bay

  Sufix: Bay

  Slave address: The address is selectable by device-tree. (TBD)

Authors:
    - Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>
    - Thalaiappan, Rathina <rathina.thalaiappan@intel.com>
    - Karanth, Ramya P <ramya.p.karanth@intel.com>

Description
===========
The Intel Edge.AI Computer Vision platforms have to be monitored using platform
devices like sensors, fan controller, IO expander etc. Some of these devices
are memory mapped and some are i2c based. Either of these devices are not
directly accessible to the host.

The host here refers to the server to which the vision accelerators are
connected over PCIe Interface. The Host needs to do a consolidated action based
on the parameters of platform devices. In general, most of the standard devices
(includes sensors, fan controller, IO expander etc) are I2C/SMBus based and are
used to provide the status of the accelerator. Standard drivers for these
devices are available based on i2c/smbus APIs.

Instead of changing the sensor drivers to adapt to PCIe interface, a generic
i2c adapter "xlink-smbus" which underneath uses xlink as physical medium is
used. With xlink-smbus, the drivers for the platform devices doesn't need to
undergo any interface change.

High-level architecture
=======================

Accessing Onchip devices::

        -------------------                     -------------------
        |   Remote Host   |                     |   Local Host    |
        |   IA CPU        |                     | Vision platforms|
        -------------------                     -------------------
        |     Onchip      |                     |    i2c slave    | ==> Access the device
        |  sensor driver  |                     |    handler      | ==> which is mmio based
        -------------------                     -------------------
        |Intel XLINK_SMBUS|                     |Intel XLINK_SMBUS|
        |     adpater     |                     |     adapter     |
        |    (Master)     |                     |   (I2C_SLAVE)   |
        -------------------                     -------------------
        |      XLINK      |    <==========>     |     XLINK       |
        -------------------        PCIE         -------------------

Accessing Onboard devices::

        -------------------                     ----------------------
        |   Remote Host   |                     |     Local Host     |
        |   IA CPU        |                     |  Vision platforms  |
        -------------------                     ----------------------
        |    On board     |                     |      i2c smbus     | ==> Access the device
        |  sensor driver  |                     |   xfer [synopsys]  | ==> which is on i2c bus
        -------------------                     ----------------------
        |Intel XLINK_SMBUS|                     | Intel XLINK_SMBUS  |
        |     adpater     |                     |       adapter      |
        |    (Master)     |                     |(SMBUS_PROXY Master)|
        -------------------                     ----------------------
        |      XLINK      |    <==========>     |        XLINK       |
        -------------------        PCIE         ----------------------
