
#ifndef __LINUX_USB_TYPEC_H
#define __LINUX_USB_TYPEC_H

#include <linux/device.h>
#include <linux/types.h>

struct typec_port;

enum typec_port_type {
	TYPEC_PORT_DFP,
	TYPEC_PORT_UFP,
	TYPEC_PORT_DRP,
};

enum typec_partner_type {
	TYPEC_PARTNER_USB,
	TYPEC_PARTNER_CHARGER,
	TYPEC_PARTNER_ALTMODE,
	TYPEC_PARTNER_ACCESSORY,
};

enum typec_plug_type {
	USB_PLUG_NONE,
	USB_PLUG_TYPE_A,
	USB_PLUG_TYPE_B,
	USB_PLUG_TYPE_C,
	USB_PLUG_CAPTIVE,
};

enum typec_data_role {
	TYPEC_DEVICE,
	TYPEC_HOST,
};

enum typec_role {
	TYPEC_SINK,
	TYPEC_SOURCE,
};

enum typec_pwr_opmode {
	TYPEC_PWR_MODE_USB,
	TYPEC_PWR_MODE_1_5A,
	TYPEC_PWR_MODE_3_0A,
	TYPEC_PWR_MODE_PD,
};

enum typec_accessory {
	TYPEC_ACCESSORY_NONE,
	TYPEC_ACCESSORY_AUDIO,
	TYPEC_ACCESSORY_DEBUG,
	TYPEC_ACCESSORY_DAUDIO,
};

/*
 * struct typec_mode - Individual Mode of an Alternate Mode
 * @vdo: VDO returned by Discover Modes USB PD command
 * @desc: Mode description
 * @active: Tells if the mode is currently entered or not
 * @index: Index of the mode
 * @group_name: Name for the sysfs folder in form "mode<index>"
 * @group: The sysfs group (folder) for the mode
 * @attrs: The attributes for the sysfs group
 * @vdo_attr: Device attribute to expose the VDO of the mode
 * @desc_attr: Device attribute to expose the description of the mode
 * @active_attr: Device attribute to expose active of the mode
 * @roles: Only for ports. DRP if the mode is awailable in both roles
 * @roles_attr: Device attribute, only for ports, to expose the supported roles
 *
 * Details about a mode of an Alternate Mode which a connector, cable plug or
 * partner supports. Every mode will have it's own sysfs group. The details are
 * the VDO returned by discover modes command, description for the mode and
 * active flag telling is the mode currently active or not.
 */
struct typec_mode {
	u32			vdo;
	char			*desc;
	unsigned int		active:1;
	/* Only for ports */
	enum typec_port_type	roles;

	struct typec_altmode	*alt_mode;

	int			index;
	char			group_name[8];
	struct attribute_group	group;
	struct attribute	*attrs[5];
	struct device_attribute vdo_attr;
	struct device_attribute desc_attr;
	struct device_attribute active_attr;
	/* Only for ports */
	struct device_attribute roles_attr;
};

/*
 * struct typec_altmode - USB Type-C Alternate Mode
 * @dev: struct device instance
 * @name: Name for the Alternate Mode (optional)
 * @svid: Standard or Vendor ID
 * @n_modes: Number of modes
 * @modes: Array of modes supported by the Alternat Mode
 * @mode_groups: The modes as attribute groups to be exposed in sysfs
 *
 * Representation of an Alternate Mode that has SVID assigned by USB-IF. The
 * array of modes will list the modes of a particular SVID that are supported by
 * a connector, partner of a cable plug.
 */
struct typec_altmode {
	struct device		dev;
	char			*name;

	u16			svid;
	int			n_modes;
	struct typec_mode	*modes;

	const struct attribute_group **mode_groups;
};

#define to_altmode(d) container_of(d, struct typec_altmode, dev)

struct typec_port *typec_altmode2port(struct typec_altmode *);

void typec_altmode_update_active(struct typec_altmode *alt, int mode,
				 bool active);

int typec_register_altmodes(struct device *, struct typec_altmode *);
void typec_unregister_altmodes(struct device *);

/*
 * struct typec_plug - USB Type-C Cable Plug
 * @dev: struct device instance
 * @index: 1 for the plug connected to DFP and 2 for the plug connected to UFP
 * @alt_modes: Alternate Modes the cable plug supports (null terminated)
 *
 * Represents USB Type-C Cable Plug.
 */
struct typec_plug {
	struct device		dev;
	int			index;
	struct typec_altmode	*alt_modes;
};

/*
 * struct typec_cable - USB Type-C Cable
 * @dev: struct device instance
 * @type: The plug type from USB PD Cable VDO
 * @usb_pd: Electronically Marked Cable
 * @active: Is the cable active or passive
 * @sop_pp_controller: Tells whether both cable plugs are configurable or not
 * @plug: The two plugs in the cable.
 *
 * Represents USB Type-C Cable attached to USB Type-C port. Two plugs are
 * created if the cable has SOP Double Prime controller as defined in USB PD
 * specification. Otherwise only one will be created if the cable is active. For
 * passive cables no plugs are created.
 */
struct typec_cable {
	struct device		dev;
	enum typec_plug_type	type;
	u32			vdo;
	unsigned int		usb_pd:1;
	unsigned int		active:1;
	unsigned int		sop_pp_controller:1;

	struct typec_plug	plug[2];
};

/*
 * struct typec_partner - USB Type-C Partner
 * @dev: struct device instance
 * @type: Normal USB device, charger, Alternate Mode or Accessory
 * @usb_pd: USB Power Delivery support
 * @vdo: VDO returned by Discover Identity USB PD command
 * @alt_modes: Alternate Modes the partner supports (null terminated)
 *
 * Details about a partner that is attached to USB Type-C port.
 */
struct typec_partner {
	struct device		dev;
	enum typec_partner_type	type;
	unsigned int		usb_pd:1;
	u32			vdo;
	enum typec_accessory	accessory;
	struct typec_altmode	*alt_modes;
};

/*
 * struct typec_capability - USB Type-C Port Capabilities
 * @role: DFP (Host-only), UFP (Device-only) or DRP (Dual Role)
 * @usb_pd: USB Power Delivery support
 * @accessory: Supported Accessory Modes
 * @num_accessory: Number of supported Accessory Modes
 * @alt_modes: Alternate Modes the connector supports (null terminated)
 * @try_role: Set a fixed data role for DRP port
 * @dr_set: Set Data Role
 * @pr_set: Set Power Role
 * @vconn_set: Set VCONN Role
 * @activate_mode: Enter/exit given Alternate Mode
 *
 * Static capabilities of a single USB Type-C port.
 */
struct typec_capability {
	enum typec_port_type	type;
	unsigned int		usb_pd:1;
	enum typec_accessory	*accessory;
	unsigned int		num_accessory;
	struct typec_altmode	*alt_modes;

	int			(*try_role)(const struct typec_capability *,
					    enum typec_role);

	int			(*dr_set)(const struct typec_capability *,
					  enum typec_data_role);
	int			(*pr_set)(const struct typec_capability *,
					  enum typec_role);
	int			(*vconn_set)(const struct typec_capability *,
					     enum typec_role);

	int			(*activate_mode)(struct typec_altmode *,
						 int mode, int activate);
};

/*
 * struct typec_connection - Details about USB Type-C port connection event
 * @partner: The attached partner
 * @cable: The attached cable
 * @data_role: Initial USB data role (host or device)
 * @pwr_role: Initial Power role (source or sink)
 * @vconn_role: Initial VCONN role (source or sink)
 * @pwr_opmode: The power mode of the connection
 *
 * All the relevant details about a connection event. Wrapper that is passed to
 * typec_connect(). The context is copied when typec_connect() is called and the
 * structure is not used for anything else.
 */
struct typec_connection {
	struct typec_partner	*partner;
	struct typec_cable	*cable;

	enum typec_data_role	data_role;
	enum typec_role		pwr_role;
	enum typec_role		vconn_role;
	enum typec_pwr_opmode	pwr_opmode;
};

struct typec_port *typec_register_port(struct device *dev,
				       const struct typec_capability *cap);
void typec_unregister_port(struct typec_port *port);

int typec_connect(struct typec_port *port, struct typec_connection *con);
void typec_disconnect(struct typec_port *port);

/* Callbacks from driver */

void typec_set_data_role(struct typec_port *, enum typec_data_role);
void typec_set_pwr_role(struct typec_port *, enum typec_role);
void typec_set_vconn_role(struct typec_port *, enum typec_role);
void typec_set_pwr_opmode(struct typec_port *, enum typec_pwr_opmode);

#endif /* __LINUX_USB_TYPEC_H */
