// SPDX-License-Identifier: GPL-2.0-only
/*
 * HDDL Device Kernel module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <asm/page.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <uapi/linux/stat.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/xlink.h>
#include <linux/time.h>
#include <linux/kmod.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/hddl_device.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/xlink_drv_inf.h>
#define DRIVER_NAME "hddl_device"

#define HOST_KMB_TJ_DEVICE "host_kmb_tj"
#define SLAVE_KMB_TJ_DRIVER "slave_kmb_tj"

#define XLINK_MAX_DEVICE_NAME_SIZE 128
#define XLINK_MAX_DEVICE_LIST_SIZE 8

#ifndef CONFIG_HDDL_LOCAL_HOST

struct mutex my_mutex; /* shared between the threads */

static struct i2c_board_info soc_i2c_ext_devices[] = {
	{
		I2C_BOARD_INFO("tmp112", 0x48),
	},
	{
		I2C_BOARD_INFO("tmp112", 0x49),
	},
	{
		I2C_BOARD_INFO("ina3221", 0x40),
	},
	{
		I2C_BOARD_INFO("emc2103", 0x2e),
	},
};
static struct i2c_board_info host_xlinki2c_devices[] = {
	{
		I2C_BOARD_INFO("host_kmb_tj", 0x5a),
	},
};
#else
static struct i2c_board_info kmb_i2c_devices[] = {
	{
		I2C_BOARD_INFO("slave-kmb-tj", (unsigned short)0x4000005b),
	},
};
#endif

#define HDDL_DEVICE_DEBUG 0

struct hddl_device_kmb kmb_hddls[HDDL_MAX_DEVICE] = {0};
#define HDDL_NODE_XLINK_CHANNEL 1080
#define HDDL_I2C_XLINK_CHANNEL 1081

struct device_driver hddl_dbg_name = {
	.name = "hddldev"
};

struct device hddl_dbg_subname = {
	.init_name = "",	/* set to "" */
	.driver = &hddl_dbg_name
};

static struct device *hddl_dbg = &hddl_dbg_subname;


#ifdef CONFIG_HDDL_LOCAL_HOST

uint32_t hddl_board_id;
uint32_t hddl_kmb_id;


int hddl_get_board_id(void)
{
	return hddl_board_id;
}
EXPORT_SYMBOL(hddl_get_board_id);

int hddl_get_kmb_id(void)
{
	return hddl_kmb_id;
}
EXPORT_SYMBOL(hddl_get_kmb_id);

int hddl_get_i2c_slave_address(void)
{
	return ((hddl_board_id << 3) | hddl_kmb_id);
}
EXPORT_SYMBOL(hddl_get_i2c_slave_address);

static int hddl_device_thread_create(void);

static int __init hddl_device_init(void)
{
	char *gpio_base_address; // = NULL;
	uint32_t board_id;
	uint32_t kmb_id;

	//int gpio_base_address1;
	gpio_base_address = ioremap(0x20320000, 2048);

	/* Configure the GPIOs */

	writel(0x1C0F, gpio_base_address + 0x2CC);
	writel(0x1C0F, gpio_base_address + 0x2D0);
	writel(0x1C0F, gpio_base_address + 0x2D4);

#if HDDL_DEVICE_DEBUG
	dev_info(hddl_dbg, "0x2CC = %x\n", readl(gpio_base_address + 0x2CC));
	dev_info(hddl_dbg, "0x2D0 = %x\n", readl(gpio_base_address + 0x2D0));
	dev_info(hddl_dbg, "0x2D4 = %x\n", readl(gpio_base_address + 0x2D4));
#endif

	writel(0x1C0F, gpio_base_address + 0x328);
	writel(0x1C0F, gpio_base_address + 0x32C);
	writel(0x1C0F, gpio_base_address + 0x330);


#if HDDL_DEVICE_DEBUG
	dev_info(hddl_dbg, "0x328 = %x\n", readl(gpio_base_address + 0x328));
	dev_info(hddl_dbg, "0x32C = %x\n", readl(gpio_base_address + 0x32C));
	dev_info(hddl_dbg, "0x330 = %x\n", readl(gpio_base_address + 0x330));
#endif

	board_id = readl(gpio_base_address + 0x24);
	board_id = (board_id >> 19) & 0x7;
	kmb_id = readl(gpio_base_address + 0x28);
	kmb_id = (kmb_id >> 10) & 0x7;
	if (kmb_id > 2) {
		kmb_id = 0;
		dev_info(hddl_dbg, "HDDL: GPIO KEEMBAY ID > 2, ");
		dev_info(hddl_dbg, "Hence setting KEEMBAY ID = 0\n");
	}

	kmb_hddls[board_id].board_id = board_id;
	kmb_hddls[board_id].soc[kmb_id].id = (board_id << 4) | kmb_id;
	hddl_board_id = board_id;
	hddl_kmb_id = kmb_id;

#if HDDL_DEVICE_DEBUG
	dev_info(hddl_dbg, "HDDL:Board Id = %x\n", hddl_board_id);
	dev_info(hddl_dbg, "HDDL:Kmb Id = %x\n", hddl_kmb_id);
#endif

	hddl_device_thread_create();
	return 0;
}

static void hddl_device_exit(void)
{
	dev_info(hddl_dbg, "%s done\n", __func__);
}

#endif

/*ioctl*/
dev_t dev;
int sw_id_value;
static struct class *dev_class;
static struct cdev hddl_cdev;

static long hddl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	int i, j, rc = 0;
	struct sw_id_hddl_data data = {0};
	struct sw_id_soft_reset reset = {0};
	struct xlink_handle *devH;

	switch (cmd) {
	case HDDL_SOFT_RESET:
		if (copy_from_user(&reset,
			(int32_t *)arg, sizeof(struct sw_id_soft_reset)))
			return -EFAULT;
		for (i = 0; i < HDDL_MAX_DEVICE; i++) {
			for (j = 0; j < HDDL_MAX_BAY_DEVICE; j++) {
				if (kmb_hddls[i].soc[j].devH.sw_device_id
					== reset.sw_id) {
					break;
				}
			}
			if (j < HDDL_MAX_BAY_DEVICE)
				break;
		}
		if (i == HDDL_MAX_DEVICE) {
			dev_warn(hddl_dbg, "SoftReset: [%x]device not found:\n",
				 reset.sw_id);
			return -EFAULT;
		}

		/* xlink-reset */
		devH = &kmb_hddls[i].soc[j].devH;
		rc =  xlink_reset_device(devH);

		if (rc > 0)
			dev_info(hddl_dbg, "xlink_reset_device failed");
		else
			reset.return_id = 1;

		if (copy_to_user((struct sw_id_soft_reset *) arg,
			&reset, sizeof(struct sw_id_soft_reset)))
			return -EFAULT;
		/* xlink-rest */
		break;
	case HDDL_READ_SW_ID_DATA:
		if (copy_from_user(&data, (int32_t *)arg,
				sizeof(struct sw_id_hddl_data)))
			return -EFAULT;
		for (i = 0; i < HDDL_MAX_DEVICE; i++) {
			for (j = 0; j < HDDL_MAX_BAY_DEVICE; j++) {
				if (kmb_hddls[i].soc[j].devH.sw_device_id
					== data.sw_id) {
					break;
				}
			}
			if (j < HDDL_MAX_BAY_DEVICE)
				break;
		}
		if (i == HDDL_MAX_DEVICE) {
			dev_warn(hddl_dbg, "HDDL Data: [%x]device not found:\n",
				 data.sw_id);
			return -EFAULT;
		}

		data.board_id = kmb_hddls[i].board_id;
		data.soc_id = kmb_hddls[i].soc[j].id;
		data.soc_adaptor_no[0] = kmb_hddls[i].soc[j].adap[0].nr;
		data.soc_adaptor_no[1] = kmb_hddls[i].soc[j].adap[1].nr;
		data.return_id = 1;
		if (copy_to_user((struct sw_id_hddl_data *) arg,
				&data,
				sizeof(struct sw_id_hddl_data)))
			return -EFAULT;
		break;
	}
	return 0;
}

static const struct file_operations fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl = hddl_ioctl,
};

int hddl_per_device_connect_thread(void *thread_param);
		/*ioctl-end*/
void hddl_device_probe(uint32_t swdev)
{
	int i, j;
	struct task_struct *task_recv;

	for (i = 0; i < HDDL_MAX_DEVICE; i++) {
		for (j = 0; j < HDDL_MAX_BAY_DEVICE; j++) {
			if (kmb_hddls[i].soc[j].devH.sw_device_id == swdev)
				break;
		}
		if (j < HDDL_MAX_BAY_DEVICE)
			break;
	}
	if (i == HDDL_MAX_DEVICE) {
		dev_warn(hddl_dbg, "Probe: [%x]device not found:\n", swdev);
		return;
	}

	task_recv = kthread_run(hddl_per_device_connect_thread,
			(void *)&kmb_hddls[i].soc[j].devH,
			"hddl_per_device_thread");
	if (task_recv == NULL)
		dev_info(hddl_dbg,
		    "hddl_device_init Thread creation failed");
}

void hddl_device_remove(uint32_t swdev)
{
	int i, j, k, rc = 0;
	uint32_t chan_num;
	struct kmb *soc;

	for (i = 0; i < HDDL_MAX_DEVICE; i++) {
		for (j = 0; j < HDDL_MAX_BAY_DEVICE; j++) {
			if (kmb_hddls[i].soc[j].devH.sw_device_id == swdev)
				break;
		}
		if (j < HDDL_MAX_BAY_DEVICE)
			break;
	}
	if (i == HDDL_MAX_DEVICE) {
		dev_warn(hddl_dbg, "Remove:[%x]device not found:\n", swdev);
		return;
	}

	soc = &kmb_hddls[i].soc[j];
	for (k = 0; k < soc->i2c_slaves_cnt; k++)
		i2c_unregister_device(soc->i2c_slaves_ext[k]);

	for (k = 0; k < soc->soc_xlinki2c_cnt; k++) {
		if (soc->soc_smbus[k])
			i2c_unregister_device(soc->soc_smbus[k]);
	}
	for (k = 0; k < soc->soc_xlinki2c_cnt; k++) {
		if (soc->soc_xlinki2c[k])
			i2c_unregister_device(soc->soc_xlinki2c[k]);
	}

	chan_num = HDDL_NODE_XLINK_CHANNEL;

	for (k = 0; k < ARRAY_SIZE(soc->xlink_i2c_plt_dev); k++)
		platform_device_unregister(soc->xlink_i2c_plt_dev[k]);

	rc = xlink_close_channel(&soc->devH, chan_num);

	rc = xlink_disconnect(&soc->devH);

	dev_info(hddl_dbg, "[%x]kmb device down\n", swdev);
}

uint32_t xlink_device_events[] = {
	_NOTIFY_INCOMING_DISCONNECTION,
	_NOTIFY_DEVICE_DISCONNECTED,
	_NOTIFY_DEVICE_CONNECTED,
	_ERROR_UNEXPECTED_DISCONNECTION,
	_NUM_EVENT_TYPE
};


int hddl_event_notify(uint32_t swdevid, enum _xlink_device_event_type event)
{
	dev_info(hddl_dbg, "xlink pcie notify[%x]: [%d]\n", swdevid, event);
	switch (event) {
	/*case _NOTIFY_INCOMING_DISCONNECTION:*/
	case _NOTIFY_DEVICE_DISCONNECTED:
	case _ERROR_UNEXPECTED_DISCONNECTION:
		hddl_device_remove(swdevid);
		break;

	case _NOTIFY_DEVICE_CONNECTED:
		hddl_device_probe(swdevid);
		break;

	default:
		dev_info(hddl_dbg, "HDDL:pcie notify-Error[%x]:[%d]\n",
			swdevid,
			event);
		break;
	}
	return 0;
}

int hddl_per_device_connect_thread(void *thread_param)
{
	struct kmb *soc;
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t stat = 0xFF;
	uint32_t board_info;
	uint32_t board_id_rcvd;
	uint32_t size = 0;
	uint32_t rc = X_LINK_ERROR;
	uint32_t board_id = 0;
	uint32_t kmb_id = 0;
	struct timespec64 ts;
	uint32_t chan_num;
	uint32_t j = 0, i = 0;
#ifndef CONFIG_HDDL_LOCAL_HOST
	struct i2c_adapter *temp;
	struct i2c_board_info temp_host_i2c_device;
#endif
	struct xlink_handle *devH = thread_param;
	struct platform_device_info xlink_i2c_info = {0};

	rc = xlink_get_device_name(devH,
		device_name, XLINK_MAX_DEVICE_NAME_SIZE);
	if (rc > 0) {
		dev_info(hddl_dbg,
			"HDDL:Failed to get device name of id [EC%d] %x\n",
			rc, devH->sw_device_id);
		return rc;
	}
	rc = X_LINK_ERROR;
	while (rc != X_LINK_SUCCESS) {
		rc = xlink_boot_device(devH, device_name);
		msleep_interruptible(1000);
	}

	rc = X_LINK_ERROR;
	while (rc != X_LINK_SUCCESS)
		rc = xlink_get_device_status(devH, &stat);

	rc = X_LINK_ERROR;
	while (rc  != X_LINK_SUCCESS)
		rc = xlink_connect(devH);

	chan_num = HDDL_NODE_XLINK_CHANNEL;

	xlink_pcie_register_device_event(devH->sw_device_id,
				xlink_device_events,
				_NUM_EVENT_TYPE,
				hddl_event_notify,
				0);

	while ((rc = xlink_open_channel(devH,
			chan_num,
			RXB_TXB, /* mode */
			64 * 1024,
			0 /* timeout */)) != 0) {
	}
	dev_info(hddl_dbg, "Booting Device..%x", devH->sw_device_id);

#ifdef CONFIG_HDDL_LOCAL_HOST
	size = sizeof(ts);
	rc = xlink_read_data_to_buffer(devH, chan_num, (uint8_t *)&ts, &size);
	xlink_release_data(devH, chan_num, NULL);

	do_settimeofday64(&ts);

	board_id = hddl_get_board_id();
	kmb_id = hddl_get_kmb_id();
	board_info = (board_id << 4) | kmb_id;
	rc = xlink_write_volatile(devH, chan_num,
		(uint8_t *)&board_info, sizeof(board_info));

	size = sizeof(board_id_rcvd);
	rc = xlink_read_data_to_buffer(devH, chan_num,
		(uint8_t *) &board_id_rcvd, &size);
	xlink_release_data(devH, chan_num, NULL);

	if (board_info == ~(board_id_rcvd)) {
		memcpy(&kmb_hddls[board_id].soc[kmb_id].devH, devH,
		sizeof(struct xlink_handle));
	}
#else

	ktime_get_real_ts64(&ts);
	rc = xlink_write_volatile(
		devH,
		chan_num,
		(uint8_t *) &ts,
		sizeof(struct timespec64));
	size = sizeof(board_id_rcvd);
	rc = xlink_read_data_to_buffer(devH, chan_num,
		(uint8_t *)&board_id_rcvd, &size);
	xlink_release_data(devH, chan_num, NULL);

	board_info = ~(board_id_rcvd);
	rc = xlink_write_volatile(devH, chan_num, (uint8_t *)&board_info,
			sizeof(board_info));
	board_id = (board_id_rcvd >> 4) & 0xF;
	kmb_id = (board_id_rcvd & 0xF);
	kmb_hddls[board_id].board_id = board_id;
	kmb_hddls[board_id].soc[kmb_id].id = board_id_rcvd;
	memcpy(&kmb_hddls[board_id].soc[kmb_id].devH, devH,
			sizeof(struct xlink_handle));
#endif
	soc = &kmb_hddls[board_id].soc[kmb_id];
	for (j = 0; j < 2; ++j) {
		xlink_i2c_info.name = "i2c_xlink";
		xlink_i2c_info.id = board_id << 4 | kmb_id << 2 | j;
		kmb_hddls[board_id].soc[kmb_id].xlink_i2c_ch[j] =
					HDDL_I2C_XLINK_CHANNEL + j;
		xlink_i2c_info.data = &kmb_hddls[board_id].soc[kmb_id];
		xlink_i2c_info.size_data =
				sizeof(kmb_hddls[board_id].soc[kmb_id]);
		soc->xlink_i2c_plt_dev[j] =
		platform_device_register_full(&xlink_i2c_info);
		soc->adap[j] =
			*((struct i2c_adapter *)platform_get_drvdata(
						soc->xlink_i2c_plt_dev[j]));
	}

	dev_info(hddl_dbg, "Handshake Complete = %x\n", devH->sw_device_id);
	dev_info(hddl_dbg,
		"Board[%x] Soc[%x] Adapter[%d]\n",
		kmb_hddls[board_id].board_id,
		kmb_hddls[board_id].soc[kmb_id].id,
		soc->adap[0].nr);

#ifndef CONFIG_HDDL_LOCAL_HOST
	soc->soc_xlinki2c_cnt
		= sizeof(host_xlinki2c_devices)/sizeof(struct i2c_board_info);

	soc->soc_smbus
		= kzalloc(
			sizeof(struct i2c_client *) * (soc->soc_xlinki2c_cnt),
			GFP_KERNEL);

	soc->soc_xlinki2c
		= kzalloc(
			sizeof(struct i2c_client *) * (soc->soc_xlinki2c_cnt),
			GFP_KERNEL);

	for (i = 0; i < soc->soc_xlinki2c_cnt; i++) {
		temp_host_i2c_device = host_xlinki2c_devices[i];

		if (board_id <= 4) {
			/* slave address range 0x10 -- 0x1F */
			temp_host_i2c_device.addr =
						kmb_id + 0x10 + (board_id * 3);
		} else {
			/* Slave address range 0x60 -- 0x6F */
			temp_host_i2c_device.addr =
				kmb_id + 0x60 + ((board_id - 5) * 3);
		}
		temp_host_i2c_device.platform_data = &(soc->devH.sw_device_id);

		j = 0;
		while ((temp = i2c_get_adapter(j)) != NULL) {
			if (strstr(temp->name, "SMBus I801") != NULL) {
				soc->soc_smbus[i]
				  = i2c_new_device(temp, &temp_host_i2c_device);
				break;
			}
			j++;
		}
		soc->soc_xlinki2c[i] = i2c_new_device(i2c_get_adapter(
				soc->adap[1].nr),
				&temp_host_i2c_device);
	}

	soc->i2c_slaves_cnt
		= sizeof(soc_i2c_ext_devices)/sizeof(struct i2c_board_info);

	soc->i2c_slaves_ext
		= kzalloc(
			sizeof(struct i2c_client *) * (soc->i2c_slaves_cnt),
			GFP_KERNEL);

	mutex_lock(&my_mutex);

	for (i = 0; i < soc->i2c_slaves_cnt; i++) {
		soc->i2c_slaves_ext[i] = i2c_new_device(
			i2c_get_adapter(soc->adap[0].nr),
			&soc_i2c_ext_devices[i]);

		ssleep(1);
	}
	mutex_unlock(&my_mutex);

#else
	soc->soc_xlinki2c_cnt
		= sizeof(kmb_i2c_devices)/sizeof(struct i2c_board_info);
	soc->soc_xlinki2c
		= kzalloc(
			sizeof(struct i2c_client *) * (soc->soc_xlinki2c_cnt),
		GFP_KERNEL);
	for (i = 0; i < soc->soc_xlinki2c_cnt; i++) {
		soc->soc_xlinki2c[i] = i2c_new_device(
			i2c_get_adapter(soc->adap[1].nr),
			&kmb_i2c_devices[i]);
	}
#endif
	dev_info(hddl_dbg, "[%x]kmb device up\n", devH->sw_device_id);
	return 0;
}
EXPORT_SYMBOL_GPL(hddl_per_device_connect_thread);




#define SW_DEVICE_ID_INTERFACE_SHIFT 24U
#define SW_DEVICE_ID_INTERFACE_MASK  0x7
#define GET_INTERFACE_FROM_SW_DEVICE_ID(id) \
	((id >> SW_DEVICE_ID_INTERFACE_SHIFT) & SW_DEVICE_ID_INTERFACE_MASK)
#define SW_DEVICE_ID_IPC_INTERFACE  0x0
#define SW_DEVICE_ID_PCIE_INTERFACE 0x1

#define XLINK_CONNECT_THREAD_BASED
static struct xlink_handle xdevH[HDDL_MAX_DEVICE*HDDL_MAX_BAY_DEVICE];
static int hddl_device_thermal_init(void *thread_param)
{
	uint32_t num_devices = 0;
	uint32_t sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t i = 0;

	dev_info(hddl_dbg, "HDDL:Getting device list...\n");
#ifndef CONFIG_HDDL_LOCAL_HOST
	mutex_init(&my_mutex);
#endif

	xlink_get_device_list(sw_device_id_list, &num_devices);
	if (num_devices == 0) {
		dev_info(hddl_dbg, "HDDL:No devices found\n");
		return 0;
	} else if (num_devices > (HDDL_MAX_DEVICE*HDDL_MAX_BAY_DEVICE)) {
		dev_info(hddl_dbg, "HDDL:Increase xdevH array size\n");
		return -1;
	}
	dev_info(hddl_dbg, "%u devices found...\n", num_devices);
	for (i = 0; i < num_devices; i++) {
		memset(&xdevH[i], 0, sizeof(struct xlink_handle));

		xdevH[i].dev_type = HOST_DEVICE;
		xdevH[i].sw_device_id = sw_device_id_list[i];
		xlink_get_device_name(&(xdevH[i]),
		device_name, XLINK_MAX_DEVICE_NAME_SIZE);
		dev_info(hddl_dbg, "HDDL:Device name: %x %s\n",
		xdevH[i].sw_device_id, device_name);
	}

	for (i = 0; i < num_devices; i++) {
		struct task_struct *task_recv;

		if (GET_INTERFACE_FROM_SW_DEVICE_ID(xdevH[i].sw_device_id)
				== SW_DEVICE_ID_PCIE_INTERFACE) {
			task_recv = kthread_run(hddl_per_device_connect_thread,
					(void *)&xdevH[i],
					"hddl_per_device_thread");
			if (task_recv == NULL)
				dev_info(hddl_dbg, "Thread creation failed");
		}

	}
	return 0;
}

#ifndef CONFIG_HDDL_LOCAL_HOST
static void hddl_device_thermal_exit(void)
{
	/*ioctl*/
	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&hddl_cdev);
	unregister_chrdev_region(dev, 1);
	dev_info(hddl_dbg, "Device Driver Remove...Done!!!\n");

	/*ioctl-end*/
	return;
};
#endif

static int hddl_device_thread_create(void)
{
	struct task_struct *task_recv;

	/* create receiver thread */
	task_recv = kthread_run(hddl_device_thermal_init, NULL,
		"hddl_device receiver_thread");
	if (task_recv == NULL)
		dev_info(hddl_dbg, "hddl_device_init Thread creation failed");
		/*ioctl*/

  /*Allocating Major number*/
	if ((alloc_chrdev_region(&dev, 0, 1, "Hddl_Dev")) < 0) {
		dev_info(hddl_dbg, "Cannot allocate major number\n");
		return -1;
	}
	dev_info(hddl_dbg, "Major = %d Minor = %d\n", MAJOR(dev), MINOR(dev));
	/*Creating cdev structure*/
	cdev_init(&hddl_cdev, &fops);
	/*Adding character device to the system*/
	if ((cdev_add(&hddl_cdev, dev, 1)) < 0) {
		dev_info(hddl_dbg, "Cannot add the device to the system\n");
		goto r_class;
	}
	/*Creating struct class*/
	dev_class = class_create(THIS_MODULE, "hddl_class");
	if (dev_class == NULL) {
		dev_info(hddl_dbg, "Cannot create the struct class\n");
		goto r_class;
	}
	/*Creating device*/
	if ((device_create(dev_class, NULL, dev, NULL, "hddl_dev"))
		== NULL) {
		dev_info(hddl_dbg, "Cannot create the Device 1\n");
		goto r_device;
	}
	return 0;
r_device:
	class_destroy(dev_class);
r_class:
	unregister_chrdev_region(dev, 1);

/*ioctl-end*/
	return 0;
}

#ifdef CONFIG_HDDL_LOCAL_HOST
module_init(hddl_device_init);
module_exit(hddl_device_exit);
#else
module_init(hddl_device_thread_create);
module_exit(hddl_device_thermal_exit);
#endif

MODULE_DESCRIPTION("KeemBay HDDL Device driver");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_LICENSE("GPL v2");
