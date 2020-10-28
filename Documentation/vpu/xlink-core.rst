.. SPDX-License-Identifier: GPL-2.0

=============================
xLink-core software subsystem
=============================

The purpose of the xLink software subsystem is to facilitate communication
between multiple users on multiple nodes in the system.

There are three types of xLink nodes:

1. Remote Host: this is an external IA/x86 host system that is only capable of
   communicating directly to the Local Host node on VPU 2.x products.
2. Local Host: this is the ARM core within the VPU 2.x  SoC. The Local Host can
   communicate upstream to the Remote Host node, and downstream to the VPU IP
   node.
3. VPU IP: this is the Leon RT core within the VPU 2.x SoC. The VPU IP can only
   communicate upstream to the Local Host node.

xLink provides a common API across all interfaces for users to access xLink
functions and provides user space APIs via an IOCTL interface implemented in
the xLink core.

xLink manages communications from one interface to another and provides routing
of data through multiple channels across a single physical interface.

It exposes a common API across all interfaces at both kernel and user levels
for processes/applications to access.

It has typical API types (open, close, read, write) that you would associate
with a communication interface.

It also has other APIs that are related to other functions that the device can
perform, e.g. boot, reset get/set device mode.
The driver is broken down into 4 source files.

xlink-core:
Contains driver initialization, driver API and IOCTL interface (for user
space).

xlink-multiplexer:
The Multiplexer component is responsible for securely routing messages through
multiple communication channels over a single physical interface.

xlink-dispatcher:
The Dispatcher component is responsible for queueing and handling xLink
communication requests from all users in the system and invoking the underlying
platform interface drivers.

xlink-platform:
provides abstraction to each interface supported (PCIe, USB, IPC, etc).

Typical xLink transaction (simplified):
When a user wants to send data across an interface via xLink it firstly calls
xlink connect which connects to the relevant interface (PCIe, USB, IPC, etc.)
and then xlink open channel.

Then it calls xlink write function. This takes the data, passes it to the
kernel which packages up the data and channel and then adds it to a transmit
queue.

A separate thread reads this transaction queue and pops off data if available
and passes the data to the underlying interface (e.g. PCIe) write function.
Using this thread provides serialization of transactions and decouples the user
write from the platform write.

On the other side of the interface, a thread is continually reading the
interface (e.g. PCIe) via the platform interface read function and if it reads
any data it adds it to channel packet container.

The application at this side of the interface will have called xlink connect,
opened the channel and called xlink read function to read data from the
interface and if any exists for that channel, the data gets popped from the
channel packet container and copied from kernel space to user space buffer
provided by the call.

xLink can handle API requests from multi-process and multi-threaded
application/processes.

xLink maintains 4096 channels per device connected (via xlink connect) and
maintains a separate channel infrastructure for each device.
