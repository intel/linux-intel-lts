.. SPDX-License-Identifier: GPL-2.0

===============================
Kernel driver: xLink IPC driver
===============================

Supported chips:

* | Intel Edge.AI Computer Vision platforms: Keem Bay
  | Suffix: Bay
  | Datasheet: (not yet publicly available)

Introduction
============

The xLink IPC driver interfaces the xLink Core driver with the Keem Bay VPU IPC
driver, thus enabling xLink to control and communicate with the VPU IP present
on the Intel Keem Bay SoC.

Specifically the driver enables xLink Core to:

* Boot / Reset the VPU IP
* Register to VPU IP event notifications (device connected, device disconnected,
  WDT event)
* Query the status of the VPU IP (OFF, BUSY, READY, ERROR, RECOVERY)
* Exchange data with the VPU IP, using the Keem Bay IPC mechanism

  * Including the ability to send 'volatile' data (i.e. small amounts of data,
    up to 128-bytes that was not allocated in the CPU/VPU shared memory region)

Sending / Receiving 'volatile' data
===================================

Data to be exchanged with Keem Bay IPC needs to be allocated in the portion of
DDR shared between the CPU and VPU.

This can be impractical for small amounts of data that user code can allocate
on the stack.

To reduce the burden on user code, xLink Core provides special send / receive
functions to send up to 128 bytes of 'volatile data', i.e., data that is not
allocated in the shared memory and that might also disappear after the xLink
API is called (e.g., because allocated on the stack).

The xLink IPC driver implements support for transferring such 'volatile data'
to the VPU using Keem Bay IPC. To this end, the driver reserves some memory in
the shared memory region.

When volatile data is to be sent, xLink IPC allocates a buffer from the
reserved memory region and copies the volatile data to the buffer. The buffer
is then transferred to the VPU using Keem Bay IPC.
