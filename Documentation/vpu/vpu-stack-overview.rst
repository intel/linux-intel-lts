.. SPDX-License-Identifier: GPL-2.0

======================
Intel VPU architecture
======================

Overview
========

The Intel Movidius acquisition has developed a Vision Processing Unit (VPU)
roadmap of products starting with Keem Bay (KMB). The hardware configurations
the VPU can support include:

1. Standalone smart camera that does local Computer Vision (CV) processing in
   camera
2. Standalone appliance or signel board computer connected to a network and
   tethered cameras doing local CV processing
3. Embedded in a USB dongle or M.2 as an CV accelerator.
4. Multiple VPU enabled SOC's on a PCIe card as a CV accelerator in a larger IA
   box or server.

Keem Bay is the first instance of this family of products. This document
provides an architectural overview of the software stack supporting the VPU
enabled products.

Keem Bay (KMB) is a Computer Vision AI processing SoC based on ARM A53 CPU that
provides Edge neural network acceleration (inference) and includes a Vision
Processing Unit (VPU) hardware. The ARM CPU SubSystem (CPUSS) interfaces
locally to the VPU and enables integration/interfacing with a remote host over
PCIe or USB or Ethernet interfaces. The interface between the CPUSS and the VPU
is implemented with hardware FIFOs (Control) and coherent memory mapping (Data)
such that zero copy processing can happen within the VPU.

The KMB can be used in all 4 of the above classes of designs.

We refer to the 'local host' as being the ARM part of the SoC, while the
'remote host' as the IA system hosting the KMB device(s). The KMB SoC boots
from an eMMC via uBoot and ARM Linux compatible device tree interface with an
expectation to fully boot within hundreds of milliseconds. There is also
support for downloading the kernel and root file system image from a remote
host.

The eMMC can be updated with standard Mender update process.
See https://github.com/mendersoftware/mender

The VPU is started and controlled from the A53 local host. Its firmware image
is loaded using the drive firware helper KAPI's.

The VPU IP firware payload consists of a SPARC ISA RTEMS bootloader and/or
application binary.

The interface allowing (remote or local) host clients to access VPU IP
capabilities is realized through an abstracted programming model, which
provides Remote Proxy APIs for a host CPU application to dynamically create and
execute CV and NN workloads on the VPU. All frameworks exposed through
programming model’s APIs are contained in the pre-compiled standard firmware
image.

There is a significant software stack built up to support KMB and the use
cases. The rest of this documentation provides an overview of the components
of the stack.

Keem Bay IPC
============

Directly interfaces with the KMB hardware FIFOs to provide zero copy processing
from the VPU. It implements the lowest level protocol for interacting with the
VPU.

The Keem Bay IPC mechanism is based on shared memory and hardware FIFOs.
Specifically there are:

* Two 128-entry hardware FIFOs, one for the CPU and one for the VPU.
* Two shared memory regions, used as memory pool for allocating IPC buffers.

An IPC channel is a software abstraction allowing communication multiplexing,
so that multiple applications / users can concurrently communicate with the
VPU.  IPC channels area conceptually similar to socket ports.

There are a total of 1024 channels, each one identified by a channel ID,
ranging from 0 to 1023.

Channels are divided in two categories:

* High-Speed (HS) channels, having IDs in the 0-9 range.
* General-Purpose (GP) channels, having IDs in the 10-1023 range.

HS channels have higher priority over GP channels and can be used by
applications requiring higher throughput or lower latency.

Since all the channels share the same hardare resources (i.e., the hardware
FIFOs and the IPC memory pools), the Keem Bay IPC driver uses software queues
to give a higher priority to HS channels.

The driver supports a build-time configurable number of communication channels
defined in a so-called Channel Mapping Table.

An IPC channel is full duplex: a pending operation from a certain channel does
not block other operations on the same channel, regardless of their operation
mode (blocking or non-blocking).

Operation mode is individually selectable for each channel, per operation
direction (read or write). All operations for that direction comply to
selection.


Keem Bay-VPU-IPC
================

This is the MMIO driver of the VPU IP block inside the SOC. It is a control
driver mapping IPC channel communication to Xlink virtual channels.

This driver provides the following functionality to other drivers in the
communication stack:

* VPU IP execution control (firmware load, start, reset)
* VPU IP event notifications (device connected, device disconnected, WDT event)
* VPU IP device status query (OFF, BUSY, READY, ERROR, RECOVERY)
* Communication via the IPC protocol (wrapping the Keem Bay IPC driver and
  exposing it to higher level Xlink layer)

In addition to the above, the driver exposes SoC information (like stepping,
device ID, etc.) to user-space via sysfs.

This driver depends on the 'Keem Bay IPC' driver, which enables the Keem Bay
IPC communication protocol.

The driver uses the Firmware API to load the VPU firmware from user-space.

Xlink-IPC
=========
This component implements the IPC specific Xlink protocol. It maps channel
IDs to hardware FIFO entries, using the Keem Bay VPU IPC driver.

Some of the main functions this driver provides:

* establishing a connection with an IPC device
* obtaining a list with the available devices
* obtaining the status for a device
* booting a device
* resetting a device
* opening and closing channels
* issuing read and write operations

Xlink-core
==========

This component implements an abstracted set of control and communication APIs
based on channel identification. It is intended to support VPU technology both
at SoC level as well as at IP level, over multiple interfaces.

It provides symmetrical services, where the producer and the consumer have
the same privileges.

Xlink driver has the ability to abstract several types of communication
channels underneath, allowing the usage of different interfaces with the same
function calls.

Xlink services are available to both kernel and user space clients and include:

* interface abstract control and communication API
* multi device support
* concurrent communication across 4096 communication channels (from 0 to
  0xFFF), with customizable properties
* full duplex channels with multiprocess and multithread support
* channel IDs can be mapped to desired physical interface (PCIe, USB, ETH, IPC)
  via a Channel Mapping Table
* asynchronous fast passthrough mode: remote host data packets are directly
  dispatched using interrupt systems running on local host to IPC calls for low
  overhead
* channel handshaking mechanism for peer to peer communication, without the
  need of static channel preallocation
* channel resource management
* asynchronous data and device notifications to subscribers

Xlink transports: PCIe, USB, ETH, IPC, XLink-PCIe

XLink-PCIe
==========
This is an endpoint driver that maps Xlink channel IDs to PCIe channels.

This component ensures (remote)host-to-(local)host communication, and VPU IP
communication via an asynchronous passthrough mode, where PCIe data loads are
directly dispatched to Xlink-IPC.

The component builds and advertises Device IDs that are used by local host
application in case of multi device scenarios.

XLink-USB
==========
This is an endpoint driver that maps Xlink channel IDs to bidirectional
USB endpoints and supports CDC USB class protocol. More than one Xlink channels
can be mapped to a single USB endpoint.

This component ensures host-to-host communication, and, as well, asynchronous
passthrough communication, where USB transfer packets are directly dispatched
to Xlink-IPC.

The component builds and advertises Device IDs that can are used by local host
application in case of multi device scenarios.

XLink-ETH
=========

This is an endpoint driver that maps Xlink channel IDs to Ethernet
sockets.

This component ensures host-to-host communication, and, as well, asynchronous
passthrough communication, where Ethernet data loads are directly dispatched to
Xlink-IPC.

The component builds and advertises Device IDs that can are used by local host
application in case of multi device scenarios.

Assorted drivers that depend on this stack:

Xlink-SMB
=========
The Intel Edge.AI Computer Vision platforms have to be monitored using platform
devices like sensors, fan controller, IO expander etc. Some of these devices
are memory mapped and some are I2C-based. None of these devices is directly
accessible to the host.

The host here refers to the server to which the vision accelerators are
connected over PCIe Interface. The Host needs to do a consolidated action based
on the parameters of platform devices. In general, most of the standard devices
(includes sensors, fan controller, IO expander etc) are I2C/SMBus based and are
used to provide the status of the accelerator. Standard drivers for these
devices are available based on I2C/SMBus APIs.

Instead of changing the sensor drivers to adapt to PCIe interface, a generic
I2C adapter "Xlink-SMBus" which underneath uses Xlink as physical medium is
used. With Xlink-SMBus, the drivers for the platform devices don't need to
undergo any interface change.

TSEN
====

Thermal sensor driver for exporting thermal events to the local Arm64 host as
well as to the remote X86 host if in the PCIe add-in CV accelerator
configuration.

The driver receives the junction temperature from different heating points
inside the SOC. The driver will receive the temperature on SMBus connection and
forward over Xlink-smb when in a remote host configuration.

In Keem Bay, the four thermal junction temperature points are Media Subsystem
(mss), Neral Network subsystem (nce), Compute subsystem (cse) and SOC(maximum
of mss, nce and cse).

HDDL
====

- Exports details of temperature sensor, current sensor and fan controller
  present in Intel Edge.AI Computer Vision platforms to IA host.
- Enable Time sync of Intel Edge.AI Computer Vision platform with IA host.
- Handles device connect and disconnect events.
- Receives slave address from the IA host for memory mapped thermal sensors
  present in SoC (Documentation/hwmon/intel_tsens_sensors.rst).
- Registers I2C slave device for slaves present in Intel Edge.AI Computer
  Vision platform


VPUMGR (VPU Manager)
====================

Bridges firmware on VPU side and applications on CPU user-space, it assists
firmware on VPU side serving multiple user space application processes on CPU
side concurrently while also performing necessary data buffer management on
behalf of VPU IP.
