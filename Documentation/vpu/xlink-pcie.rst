.. SPDX-License-Identifier: GPL-2.0

================================
Kernel driver: Xlink-pcie driver
================================
Supported chips:
  * Intel Edge.AI Computer Vision platforms: Keem Bay
    Suffix: Bay
    Slave address: 6240
    Datasheet: Publicly available at Intel

Author: Srikanth Thokala Srikanth.Thokala@intel.com

Introduction
============
The Xlink-pcie driver provides transport layer implementation for
the data transfers to support Xlink protocol subsystem communication with the
peer device, i.e., between remote host system and Keem Bay device.

The Keem Bay device is an ARM-based SOC that includes a vision processing
unit (VPU) and deep learning, neural network core in the hardware.
The Xlink-pcie driver exports a functional device endpoint to the Keem Bay
device and supports two-way communication with the peer device.

High-level architecture
=======================
Remote Host: IA CPU
Local Host: ARM CPU (Keem Bay)::

        +------------------------------------------------------------------------+
        |  Remote Host IA CPU              | | Local Host ARM CPU (Keem Bay) |   |
        +==================================+=+===============================+===+
        |  User App                        | | User App                      |   |
        +----------------------------------+-+-------------------------------+---+
        |   XLink UAPI                     | | XLink UAPI                    |   |
        +----------------------------------+-+-------------------------------+---+
        |   XLink Core                     | | XLink Core                    |   |
        +----------------------------------+-+-------------------------------+---+
        |   XLink PCIe                     | | XLink PCIe                    |   |
        +----------------------------------+-+-------------------------------+---+
        |   XLink-PCIe Remote Host driver  | | XLink-PCIe Local Host driver  |   |
        +----------------------------------+-+-------------------------------+---+
        |-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:|:|:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:|
        +----------------------------------+-+-------------------------------+---+
        |     PCIe Host Controller         | | PCIe Device Controller        | HW|
        +----------------------------------+-+-------------------------------+---+
               ^                                             ^
               |                                             |
               |------------- PCIe x2 Link  -----------------|

This XLink PCIe driver comprises of two variants:
* Local Host driver

  * Intended for ARM CPU
  * It is based on PCI Endpoint Framework
  * Driver path: {tree}/drivers/misc/Xlink-pcie/local_host

* Remote Host driver

       * Intended for IA CPU
       * It is a PCIe endpoint driver
       * Driver path: {tree}/drivers/misc/Xlink-pcie/remote_host

XLink PCIe communication between local host and remote host is achieved through
ring buffer management and MSI/Doorbell interrupts.

The Xlink-pcie driver subsystem registers the Keem Bay device as an endpoint
driver and provides standard Linux PCIe sysfs interface:
'/sys/bus/pci/devices/xxxx:xx:xx.0/'


XLink protocol subsystem
========================
Xlink is an abstracted control and communication subsystem based on channel
identification. It is intended to support VPU technology both at SoC level as
well as at IP level, over multiple interfaces.

- The Xlink subsystem abstracts several types of communication channels
  underneath, allowing the usage of different interfaces with the
  same function call interface.
- The Communication channels are full-duplex protocol channels allowing
  concurrent bidirectional communication.
- The Xlink subsystem also supports control operations to VPU either
  from standalone local system or from remote system based on communication
  interface underneath.
- The Xlink subsystem supports the following communication interfaces:
    * USB CDC
    * Gigabit Ethernet
    * PCIe
    * IPC
