/*
 * <linux/usb/debug.h> -- USB Debug Class definitions.
 *
 * Usb debug class specific constants, based on:
 * USB 3.1 Device Class Specification for Debug Devices
 * Revision 1.0 – July 14, 2015
 * http://www.usb.org/developers/docs/usb_31_072715.zip
 *
 * Copyright (C) 2015, Intel Corporation.
 *
 * This software is distributed under the terms of the GNU General Public
 * License ("GPL") version 2, as published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_USB_DEBUG_H
#define __LINUX_USB_DEBUG_H

#include <linux/types.h>

/*
 * USB Debug Class Rev. 1.0
 * Appendix A: Debug-Device-Class Codes
 * Table 8-2: Debug Interface Sub-Class Code (SC_DEBUG)
 */
#define USB_SUBCLASS_DBC		0x02
#define USB_SUBCLASS_DBC_DFX		0x03
#define USB_SUBCLASS_DBC_TRACE		0x04
#define USB_SUBCLASS_DVC_GP		0x05
#define USB_SUBCLASS_DVC_DFX		0x06
#define USB_SUBCLASS_DVC_TRACE		0x07
#define USB_SUBCLASS_DEBUG_CONTROL	0x08

/*
 * USB Debug Class Rev. 1.0
 * Appendix A: Debug-Device-Class Codes
 * Table 8-3: Debug Interface Protocol Code (PC_DEBUG)
 */
#define DC_PROTOCOL_CODE_UNDEFINED	0x00
#define DC_PROTOCOL_VENDOR_OR_GNU	0x01

#define DC_PROTOCOL_GP_VENDOR	0x00
#define DC_PROTOCOL_GP_GNU	0x01

/*
 * USB Debug Class Rev. 1.0
 * 4.4.6 Debug-Unit Descriptor
 * Table 4-9: Debug Unit Descriptor - bmControl
 */
#define DC_CTL_SET_CFG_DATA_SG			(1 << 0)
#define DC_CTL_SET_CFG_DATA			(1 << 1)
#define DC_CTL_GET_CFG_DATA			(1 << 2)
#define DC_CTL_SET_CFG_ADDR			(1 << 3)
#define DC_CTL_GET_CFG_ADDR			(1 << 4)
#define DC_CTL_SET_OP_MODE			(1 << 7)
#define DC_CTL_GET_OP_MODE			(1 << 8)
#define DC_CTL_SET_BUFF_INFO			(1 << 11)
#define DC_CTL_GET_BUFF_INFO			(1 << 12)
#define DC_CTL_SET_RESET			(1 << 13)


/*
 * USB Debug Class Rev. 1.0
 * Appendix A: Debug-Device-Class Codes
 * Table 8-6: Debug Class-Specific Descriptor SubTypes
 */
#define DC_UNDEFINED			0x00
#define DC_INPUT_CONNECTION		0x01
#define DC_OUTPUT_CONNECTION		0x02
#define DC_DEBUG_UNIT			0x03
#define DC_DEBUG_ATTRIBUTES		0x04

/*
 * USB Debug Class Rev. 1.0
 *
 * 4.4.4 Input-Connection Descriptor
 * Table 4-7: Input Connection Descriptor - bConnectionType
 *
 * 4.4.5 Output Connection Descriptor
 * Table 4-8: Output Connection Descriptor - bConnectionType
 */
#define DC_CONNECTION_USB			0x00
#define DC_CONNECTION_DEBUG_CONTROL		0x01
#define DC_CONNECTION_DEBUG_DATA		0x02
#define DC_CONNECTION_DEBUG_DATA_CONTROL	0x03


/*
 * USB Debug Class Rev. 1.0
 * 4.4.6 Debug-Unit Descriptor
 * Table 4-11: dTraceFormat
 */
#define DC_VENDOR_FORMAT(v, f)    (((v)<<24)|(f))
/*Vendor N/A*/
#define DC_TRACE_NOT_FORMATED_PASSTHROUGH	DC_VENDOR_FORMAT(0x0, 0x0)
#define DC_TRACE_NOT_FORMATED_HEADER		DC_VENDOR_FORMAT(0x0, 0x1)
#define DC_TRACE_NOT_FORMATED_FOOTER		DC_VENDOR_FORMAT(0x0, 0x2)
#define DC_TRACE_NOT_FORMATED_GUID		DC_VENDOR_FORMAT(0x0, 0x5)
#define DC_TRACE_NOT_FORMATED_UTF8		DC_VENDOR_FORMAT(0x0, 0x6)
/*Vendor Intel*/
#define DC_TRACE_INTEL_FORMATED_VENDOR		DC_VENDOR_FORMAT(0x1, 0x0)
/*Vendor ARM*/
#define DC_TRACE_ARM_FORMATED_VENDOR		DC_VENDOR_FORMAT(0x2, 0x0)
/*Vendor ST*/
#define DC_TRACE_ST_FORMATED_VENDOR		DC_VENDOR_FORMAT(0x3, 0x0)
/*Vendor TI*/
#define DC_TRACE_TI_FORMATED_VENDOR		DC_VENDOR_FORMAT(0x4, 0x0)
/*Vendor Qualcomm*/
#define DC_TRACE_QCOMM_FORMATED_VENDOR		DC_VENDOR_FORMAT(0x5, 0x0)
/*Vendor AMD*/
#define DC_TRACE_AMD_FORMATED_VENDOR		DC_VENDOR_FORMAT(0x6, 0x0)
/*Vendor MIPI*/
#define DC_TRACE_MIPI_FORMATED			DC_VENDOR_FORMAT(0x80, 0x0)
/*Vendor Nexus*/
#define DC_TRACE_NEXUS_FORMATED			DC_VENDOR_FORMAT(0x81, 0x0)

/*
 * USB Debug Class Rev. 1.0
 * 4.4.6 Debug-Unit Descriptor
 * Table 4-9: Debug Unit Descriptor - bDebugUnitType
 */
#define DC_UNIT_TYPE_UNDEFINED			0x00
#define DC_UNIT_TYPE_DFX			0x01
#define DC_UNIT_TYPE_SELECT			0x02
#define DC_UNIT_TYPE_TRACE_ROUTE		0x03
#define DC_UNIT_TYPE_TRACE_PROC			0x04
#define DC_UNIT_TYPE_TRACE_GEN			0x05
#define DC_UNIT_TYPE_TRACE_SINK			0x06
#define DC_UNIT_TYPE_CONTROL			0x07
#define DC_UNIT_TYPE_VENDOR_FIRST		0x40
#define DC_UNIT_TYPE_VENDOR_LAST		0x5F

/*
 * USB Debug Class Rev. 1.0
 * 4.4.6 Debug-Unit Descriptor
 * Table 4-10: Debug Sub-Unit Type
 */
#define DC_UNIT_SUBTYPE_NULL			0x00
#define DC_UNIT_SUBTYPE_CPU			0x01
#define DC_UNIT_SUBTYPE_GFX			0x02
#define DC_UNIT_SUBTYPE_VIDEO			0x03
#define DC_UNIT_SUBTYPE_IMAGING			0x04
#define DC_UNIT_SUBTYPE_AUDIO			0x05
#define DC_UNIT_SUBTYPE_MODEM			0x06
#define DC_UNIT_SUBTYPE_BLUETOOTH		0x07
#define DC_UNIT_SUBTYPE_PWR_MGT			0x08
#define DC_UNIT_SUBTYPE_SECURITY		0x09
#define DC_UNIT_SUBTYPE_SENSOR			0x0A
#define DC_UNIT_SUBTYPE_BUSWATCH		0x0B
#define DC_UNIT_SUBTYPE_LOCATION		0x0C
#define DC_UNIT_SUBTYPE_TRACEZIP		0x0D
#define DC_UNIT_SUBTYPE_TAPCTL			0x0E
#define DC_UNIT_SUBTYPE_MEMACC			0x0F
#define DC_UNIT_SUBTYPE_OTHER			0x3F
#define DC_UNIT_SUBTYPE_SWLOGGER		0x40
#define DC_UNIT_SUBTYPE_SWROUTER		0x41
#define DC_UNIT_SUBTYPE_SWUNINT			0x42
#define DC_UNIT_SUBTYPE_SWCFGUNINT		0x43
#define DC_UNIT_SUBTYPE_SWDEBUGGER		0x44
#define DC_UNIT_SUBTYPE_VENDOR_FIRST		0x80
#define DC_UNIT_SUBTYPE_VENDOR_LAST		0xBF
#define DC_UNIT_SUBTYPE_STANDARDS		0xFF

/*
 * USB Debug Class Rev. 1.0
 * Appendix A: Debug-Device-Class Codes
 * Table 8-5: Debug Class-Specific Commands bRequest
 */
/*Set*/
#define DC_REQUEST_SET_CONFIG_DATA		0x01 /*S 5.4.4 T 5-6*/
#define DC_REQUEST_SET_CONFIG_DATA_SINGLE	0x02 /*S 5.4.3 T 5-4*/
#define DC_REQUEST_SET_CONFIG_ADDRESS		0x03 /*S 5.4.6 T 5-8*/
#define DC_REQUEST_SET_ALT_STACK		0x04 /*S 5.4.8 T 5-10*/
#define DC_REQUEST_SET_OPERATING_MODE		0x05 /*S 5.4.10 T 5-15*/
#define DC_REQUEST_SET_TRACE			0x06 /*S 5.4.14 T 5-23*/
#define DC_REQUEST_SET_BUFFER			0x09 /*S 5.4.16 T 5-25*/
#define DC_REQUEST_SET_RESET			0x0A /*S 5.4.18 T 5-28f*/
/*Get*/
#define DC_REQUEST_GET_CONFIG_DATA		0x81 /*S 5.4.5 T 5-7*/
#define DC_REQUEST_GET_CONFIG_DATA_SINGLE	0x82
#define DC_REQUEST_GET_CONFIG_ADDRESS		0x83 /*S 5.4.7 T 5-9*/
#define DC_REQUEST_GET_ALT_STACK		0x84 /*S 5.4.9 T 5-11*/
#define DC_REQUEST_GET_OPERATING_MODE		0x85 /*S 5.4.11 T 5-18*/
#define DC_REQUEST_GET_TRACE			0x86 /*S 5.4.15 T 5-24*/
#define DC_REQUEST_GET_INFO			0x87 /*S 5.4.12 T 5-19*/
#define DC_REQUEST_GET_ERROR			0x88 /*S 5.4.13 T 5-21*/
#define DC_REQUEST_GET_BUFFER			0x89 /*S 5.4.17 T 5-27*/

#endif /* __LINUX_USB_DEBUG_H */
