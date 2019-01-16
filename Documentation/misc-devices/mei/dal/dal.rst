.. SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause

Intel(R) Dynamic Application Loader (Intel(R) DAL)
===================================================

Introduction
=============

The Intel (R) Dynamic Application Loader (Intel (R) DAL) is a
Trusted Execution Environment (TEE) which, as a part of the
Converged Security Engine (CSE) firmware, enables users to directly
access and run small portions of their code on part of the system's
root of trust.

---

Onto this firmware, the DAL user installs a small Java applet,
called a Trusted Application (TA), or an applet. From the host
application that runs on the device's operating system, the below
interfaces are used to interact with the applet for running a small
function that needs to be run in a secure environment, outside of the
operating system.

DAL exposes two interfaces to the operating system, which serve as
the communication channels between trusted applications and host based
applications.
One from the user space, called Intel (R) DAL Host Interface, or JHI,
need one from kernel space, called Intel (R) Management Engine Interface
and Dynamic Application Loader (Intel (R) MEI DAL), or KDI.
Only user space applications can install and uninstall TAs. Both kernel
and user space applications can communicate with installed TAs.


Intel(R) MEI DAL Linux Kernel Driver
=====================================
The Intel(R) Management Engine Interface and Dynamic Application Loader
(Intel(R) MEI DAL) is a kernel component that provides both user space
and kernel space communication interfaces with the DAL client in the
CSE firmware, enabling the direct usage of DAL by Linux kernel
components.

User Space Interface
---------------------
DAL runs 3 processes:
       * DAL Security Domains Manager (DAL SDM)
           - manages the applets and security domains life cycles
       * DAL Intel Virtual Machine (DAL IVM)
           - the VM that runs the applets byte code
       * DAL Launcher
           - A place holder for future second VM and native applets
             support.
For each one of them, the driver exposes a char device
called /dev/dal{i}, while i is 0-2 respectively.

The user space interface serves as a transfer only channel between user
space applications and DAL FW; it allows sending raw messages from
user space to a DAL FW process, without any processing or modification
of the message data, and receiving back the raw messages which was
received from DAL FW.  The messages are sent using the char device
'write' function, and received using the 'read' function in accordance.
Usually this interface is used by the JHI (for more information about
JHI search dynamic-application-loader-host-interface in github).

Kernel Space Interface
-----------------------
The driver exposes API in <linux/dal.h> file, to allow kernel space
clients communicating with Intel DAL.

Below are the exposed APIs.

dal_create_session - creates a session to an installed trusted
                     application.
    Arguments:
        session_handle:   output param to hold the session handle
        ta_id:            trusted application (TA) id
        acp_pkg           ACP file of the TA
        acp_pkg_len:      ACP file length
        init_param:       init parameters to the session (optional)
        init_param_len:   length of the init parameters

    Returns:
        0 on success
        <0 on system failure
        >0 on DAL FW failure

dal_send_and_receive - sends and receives data to and from
                       trusted application
    Arguments:
        session_handle: session handle
        command_id:     command id
        input:          message to be sent
        input_len:      sent message size
        output:         An output parameter to hold a pointer
                        to the buffer which will contain the received
                        message.
                        This buffer is allocated by the driver and freed
                        by the user
        output_len:     An input and output parameter-
                           - input: the expected maximum length
                             of the received message.
                           - output: size of the received message
        response_code:  An output parameter to hold the return
                        value from the applet

    Returns:
        0 on success
        <0 on system failure
        >0 on DAL FW failure

dal_close_session - closes a session with trusted application
    Arguments:
        session_handle:    session handle

    Returns:
        0 on success
        <0 on system failure
        >0 on DAL FW failure

dal_set_ta_exclusive_access - sets client to be owner of the TA,
       so no one else (especially user space client) will be able
       to open a session to it
    Arguments:
        ta_id:             trusted application (TA) id

    Return:
        0 on success
        -ENODEV when the device can't be found
        -ENOMEM on memory allocation failure
        -EPERM when TA is owned by another client
        -EEXIST when TA is already owned by current client

dal_unset_ta_exclusive_access - unsets client from owning TA
    Arguments:
        ta_id:             trusted application (TA) id

    Return:
        0 on success
        -ENODEV when the device can't be found
        -ENOENT when TA wassn't found in exclusiveness TAs list
        -EPERM when TA is owned by another client

dal_get_version_info - return DAL driver version
    Arguments:
        version_info: output param to hold DAL driver version
                      information.

    Return:
        0 on success
        -EINVAL on incorrect input
