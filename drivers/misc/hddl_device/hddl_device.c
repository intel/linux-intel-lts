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

#define MAX_HDDL_DEVICES 8
T_HDDL_DEVICE_KMB_NODE kmb_hddls[MAX_HDDL_DEVICES] = {0};
#define HDDL_NODE_XLINK_CHANNEL 1050
#define HDDL_I2C_XLINK_CHANNEL 1080

uint32_t one_dev;
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

	*((volatile int *)(gpio_base_address + 0x2CC)) = 0x1C0F;
	*((volatile int *)(gpio_base_address + 0x2D0)) = 0x1C0F;
	*((volatile int *)(gpio_base_address + 0x2D4)) = 0x1C0F;

	printk(KERN_INFO "0x2CC = %x\n",
	*(volatile int *)(gpio_base_address + 0x2CC));
	printk(KERN_INFO "0x2D0 = %x\n",
	*(volatile int *)(gpio_base_address + 0x2D0));
	printk(KERN_INFO "0x2D4 = %x\n",
	*(volatile int *)(gpio_base_address + 0x2D4));

	*(volatile int *)(gpio_base_address + 0x328) = 0x1C0F;
	*(volatile int *)(gpio_base_address + 0x32C) = 0x1C0F;
	*(volatile int *)(gpio_base_address + 0x330) = 0x1C0F;

	printk(KERN_INFO "0x328 = %x\n",
	*(volatile int *)(gpio_base_address + 0x328));
	printk(KERN_INFO "0x32C = %x\n",
	*(volatile int *)(gpio_base_address + 0x32C));
	printk(KERN_INFO "0x330 = %x\n",
	*(volatile int *)(gpio_base_address + 0x330));

	board_id = *((volatile int*)(gpio_base_address + 0x24));
	board_id = (board_id >> 19) & 0x7;
	kmb_id = *((volatile int*)(gpio_base_address + 0x28));
	kmb_id = (kmb_id >> 10) & 0x7;
	printk(KERN_INFO "HDDL: GPIO BOARD ID = %u\n", board_id);
	printk(KERN_INFO "HDDL: GPIO KEEMBAY ID = %u\n", kmb_id);
	if (kmb_id > 2) {
		kmb_id = 0;
		one_dev = 1;
		printk(KERN_INFO "HDDL: GPIO KEEMBAY ID > 2, ");
		printk(KERN_INFO "Hence setting KEEMBAY ID = 0\n");
	}

	kmb_hddls[board_id].board_id = board_id;
	kmb_hddls[board_id].soc[kmb_id].id = (board_id << 4) | kmb_id;
	hddl_board_id = board_id;
	hddl_kmb_id = kmb_id;

	printk(KERN_INFO "HDDL:Board Id = %x\n", hddl_board_id);
	printk(KERN_INFO "HDDL:Kmb Id = %x\n", hddl_kmb_id);

	pr_info("HDDL:hddl_device_init done\n");
	hddl_device_thread_create();
	return 0;
}

static void hddl_device_exit(void)
{
	pr_info("HDDL:hddl_device_exit\n");
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
	T_SW_ID_HDDL_DATA swid_data = {0};
	T_SW_ID_SOFT_RESET soft_reset = {0};

	switch (cmd) {
	case HDDL_SOFT_RESET:
		if (copy_from_user(&soft_reset,
				(int32_t *)arg, sizeof(T_SW_ID_SOFT_RESET)))
			return -EFAULT;
		for (i = 0; i < 8; i++) {
			for (j = 0; j < 3; j++) {
				if (kmb_hddls[i].soc[j].devH.sw_device_id
						== soft_reset.sw_id) {
					/* xlink-reset */
					struct xlink_handle *devH =
						&kmb_hddls[i].soc[j].devH;
					rc =  xlink_reset_device(devH);

					printk(KERN_INFO "Value of rc = %d\n", rc);
					if (rc > 0)
						printk(KERN_INFO "HDDL : xlink_reset_device failed");
					else
						soft_reset.return_id = 1;

					if (copy_to_user((T_SW_ID_SOFT_RESET *) arg,
							&soft_reset, sizeof(T_SW_ID_SOFT_RESET)))
						return -EFAULT;
					/* xlink-rest */
				}
			}
		}
		break;
	case HDDL_READ_SW_ID_DATA:
		if (copy_from_user(&swid_data, (int32_t *)arg,
			sizeof(T_SW_ID_HDDL_DATA)))
			return -EFAULT;
		for (i = 0; i < 8; i++) {
			for (j = 0; j < 3; j++) {
				if (kmb_hddls[i].soc[j].devH.sw_device_id
					== swid_data.sw_id) {
					swid_data.board_id = kmb_hddls[i].board_id;
					swid_data.soc_id = kmb_hddls[i].soc[j].id;
					swid_data.soc_adaptor_no[0] =
						kmb_hddls[i].soc[j].adap[0].nr;
					swid_data.soc_adaptor_no[1] =
						kmb_hddls[i].soc[j].adap[1].nr;
					swid_data.return_id = 1;
				if (copy_to_user((T_SW_ID_HDDL_DATA *) arg,
					&swid_data, sizeof(T_SW_ID_HDDL_DATA)))
					return -EFAULT;
				}
			}
		}
		break;
	}
	return 0;
}

static struct file_operations fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl = hddl_ioctl,
};

		/*ioctl-end*/
int hddl_per_device_connect_thread(void *thread_param);

void hddl_device_probe(uint32_t sw_device_id)
{
	int i, j;
	struct task_struct *task_recv;
	printk(KERN_INFO "hddl_device_probe = %x\n", sw_device_id);

	for (i = 0; i < 8; i++) {
	for (j = 0; j < 3; j++) {
		if (kmb_hddls[i].soc[j].devH.sw_device_id == sw_device_id) {

			printk("inside probe if-condition sw_device_id %x\n", kmb_hddls[i].soc[j].devH.sw_device_id);

			printk("HDDL: Booting kmb device after reset\n");
			task_recv = kthread_run(hddl_per_device_connect_thread,
					(void *)&kmb_hddls[i].soc[j].devH,
					"hddl_per_device_thread");
			if (task_recv == NULL)
				printk(KERN_WARNING "hddl_device_init Thread creation failed");
			}
		}
	}
}

void hddl_device_remove(uint32_t sw_device_id)
{
	int i, j, k, rc;
	uint32_t chan_num;
	struct kmb *soc;

	printk(KERN_INFO "hddl_device_remove = %x\n", sw_device_id);

	for (i = 0; i < 8; i++) {
	for (j = 0; j < 3; j++) {
		if (kmb_hddls[i].soc[j].devH.sw_device_id == sw_device_id) {

			soc = &kmb_hddls[i].soc[j];
			for (k = 0; k < soc->i2c_slaves_cnt; k++) {
				printk("i2c_slaves_ext adapter %d", soc->i2c_slaves_ext[k]->adapter->nr);
				i2c_unregister_device(soc->i2c_slaves_ext[k]);
			}
			for (k = 0; k < soc->soc_xlinki2c_cnt; k++) {
				if (soc->soc_smbus[k]) {
					printk("soc_smbus adapter %d", soc->soc_smbus[k]->adapter->nr);
					i2c_unregister_device(soc->soc_smbus[k]);
				}
			}
			for (k = 0; k < soc->soc_xlinki2c_cnt; k++) {
				if (soc->soc_xlinki2c[k]) {
					printk("soc_xlinki2c adapter %d", soc->soc_xlinki2c[k]->adapter->nr);
					i2c_unregister_device(soc->soc_xlinki2c[k]);
				}
			}

			chan_num = HDDL_NODE_XLINK_CHANNEL;
			printk("chan_num form remove function = %d\n", chan_num);

			for (k = 0; k < 2; k++) {

				printk("HDDL : platform_device_unregister = %d\n", k);
				platform_device_unregister(soc->xlink_i2c_plt_dev[k]);
			}

			rc = xlink_close_channel(&soc->devH, chan_num);
			printk(KERN_INFO "HDDL:Close Channel Number[%x]: [%u] EC[%d]\n", sw_device_id, chan_num, rc);

			rc = xlink_disconnect(&soc->devH);
			printk(KERN_INFO "HDDL:Disconnect[%x]: EC[%d]\n", sw_device_id, rc);
			printk("value of rc from remove = %d", rc);
			}
		}
	}
}

uint32_t xlink_device_events[] = {
	_NOTIFY_INCOMING_DISCONNECTION,
	_NOTIFY_DEVICE_DISCONNECTED,
	_NOTIFY_DEVICE_CONNECTED,
	_ERROR_UNEXPECTED_DISCONNECTION,
	_NUM_EVENT_TYPE
};


int hddl_device_pcie_event_notify(uint32_t sw_device_id, enum _xlink_device_event_type event_type)
{
	printk(KERN_INFO "HDDL:xlink pcie notify[%x]: [%d]\n", sw_device_id, event_type);
	switch (event_type) {
	case _NOTIFY_INCOMING_DISCONNECTION:
	case _NOTIFY_DEVICE_DISCONNECTED:
	case _ERROR_UNEXPECTED_DISCONNECTION:
		hddl_device_remove(sw_device_id);
		break;

	case _NOTIFY_DEVICE_CONNECTED:
		hddl_device_probe(sw_device_id);
		break;

	default:
		printk(KERN_INFO "HDDL:xlink pcie notify - Error[%x]: [%d]\n", sw_device_id, event_type);
		break;
	}
	return 0;
}

int hddl_per_device_connect_thread(void *thread_param)
{
	struct kmb *soc;
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t device_status = 0xFF;
	uint32_t board_info;
	uint32_t board_id_rcvd;
	uint32_t size = 0;
	uint32_t rc = 0;
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
		printk(KERN_INFO
		"HDDL:Failed to get device name of id [EC%d] %x\n",
		rc, devH->sw_device_id);
		return rc;
	}
	printk(KERN_INFO "HDDL:Device name: %x %s\n",
		devH->sw_device_id, device_name);

	while ((rc = xlink_boot_device(devH, device_name)) != X_LINK_SUCCESS) {
		msleep_interruptible(1000);
	}

	while ((rc = xlink_get_device_status(devH, &device_status)) != X_LINK_SUCCESS) {

	}
	printk(KERN_INFO "HDDL:Device status[%d]: %u\n", rc, device_status);
	while ((rc = xlink_connect(devH)) != 0) {
	}

	chan_num = HDDL_NODE_XLINK_CHANNEL;
	printk(KERN_INFO "HDDL:Channel Number[%x]: %u\n", devH->sw_device_id, chan_num);

	xlink_pcie_register_device_event(devH->sw_device_id,
									xlink_device_events,
									_NUM_EVENT_TYPE,
									hddl_device_pcie_event_notify,
									0);


	while ((rc = xlink_open_channel(devH,
			chan_num,
			RXB_TXB, /* mode */
			64 * 1024,
			0 /* timeout */)) != 0) {
	}
	printk(KERN_INFO "HDDL: xlink_open_channel completed[%d]\n", rc);
	#ifdef CONFIG_HDDL_LOCAL_HOST
	size = sizeof(ts);
	rc = xlink_read_data_to_buffer(devH, chan_num, (uint8_t *)&ts, &size);
	xlink_release_data(devH, chan_num, NULL);

	printk(KERN_INFO "HDDL: Received %d Bytes [%d]\n", size, rc);
	printk(KERN_INFO "S[%llx] NS[%lx]\n", ts.tv_sec, ts.tv_nsec);
	do_settimeofday64(&ts);

	printk(KERN_INFO "HDDL: xlink_write_data to start...\n");
	board_id = hddl_get_board_id();
	kmb_id = hddl_get_kmb_id();
	board_info = (board_id << 4) | kmb_id;
	rc = xlink_write_volatile(devH, chan_num,
	(uint8_t *)&board_info, sizeof(board_info));
	printk(KERN_INFO "HDDL: xlink_write_data complete[%d].\n", rc);

	printk(KERN_INFO "HDDL: xlink_read_data to start...\n");
	size = sizeof(board_id_rcvd);
	rc = xlink_read_data_to_buffer(devH, chan_num,
	(uint8_t *) &board_id_rcvd, &size);
	xlink_release_data(devH, chan_num, NULL);
	printk(KERN_INFO
				"HDDL: [%d]xlink_read_data completed Rcvd Size[%d]\n", rc, size);
	printk(KERN_INFO "HDDL: Board Info[%x %x]\n", board_info, board_id_rcvd);

	if (board_info == ~(board_id_rcvd)) {
		memcpy(&kmb_hddls[board_id].soc[kmb_id].devH, devH,
		sizeof(struct xlink_handle));
		printk(KERN_INFO "HDDL: Handshake Complete = %x\n",
		board_info);
		printk(KERN_INFO
		"HDDL: Board[%x] Soc[%x] DevType[%x]\n",
			kmb_hddls[board_id].board_id,
			kmb_hddls[board_id].soc[kmb_id].id,
			kmb_hddls[board_id].soc[kmb_id].devH.dev_type
		);
	}
	#else

	ktime_get_real_ts64(&ts);
	printk(KERN_INFO "S[%llx] NS[%lx]\n", ts.tv_sec, ts.tv_nsec);
	rc = xlink_write_volatile(devH, chan_num, (uint8_t *) &ts, sizeof(struct timespec64));
	printk(KERN_INFO "HDDL: Size Transferred[%d] = %ld\n", 	rc, sizeof(struct timespec64));

	printk(KERN_INFO "HDDL: xlink_read_data to start...\n");
	size = sizeof(board_id_rcvd);
	rc = xlink_read_data_to_buffer(devH, chan_num,
	(uint8_t *)&board_id_rcvd, &size);
	xlink_release_data(devH, chan_num, NULL);
	printk(KERN_INFO "HDDL: xlink_read_data completed Rcvd Size[%d][%d]\n",
	rc, size);

	board_info = ~(board_id_rcvd);
	printk(KERN_INFO "HDDL: xlink_write_data to start...\n");
	rc = xlink_write_volatile(devH, chan_num, (uint8_t *)&board_info,
	sizeof(board_info));
	printk(KERN_INFO "HDDL: xlink_write_data complete.[%d]\n", rc);
	printk(KERN_INFO "HDDL: Board Info[%x %x]", board_id_rcvd, board_info);
	board_id = (board_id_rcvd >> 4) & 0xF;
	kmb_id = (board_id_rcvd & 0xF);
	kmb_hddls[board_id].board_id = board_id;
	kmb_hddls[board_id].soc[kmb_id].id = board_id_rcvd;
	memcpy(&kmb_hddls[board_id].soc[kmb_id].devH, devH,
	sizeof(struct xlink_handle));
	printk(KERN_INFO "HDDL: Board[%x] Soc[%x] DevType[%x]\n",
		kmb_hddls[board_id].board_id,
		kmb_hddls[board_id].soc[kmb_id].id,
		kmb_hddls[board_id].soc[kmb_id].devH.dev_type
	);
	#endif
	soc = &kmb_hddls[board_id].soc[kmb_id];
	if (one_dev == 1) {
		j = 1;
	} else {
		j = 0;
	}
	for (; j < 2; ++j) {
		xlink_i2c_info.name = "i2c_xlink";
		xlink_i2c_info.id = board_id << 4 | kmb_id << 2 | j;
		kmb_hddls[board_id].soc[kmb_id].xlink_i2c_ch[j] =
		HDDL_I2C_XLINK_CHANNEL + (kmb_id * 2) + j;
		xlink_i2c_info.data = &kmb_hddls[board_id].soc[kmb_id];
		xlink_i2c_info.size_data =
				sizeof(kmb_hddls[board_id].soc[kmb_id]);
		soc->xlink_i2c_plt_dev[j] =
		platform_device_register_full(&xlink_i2c_info);
		soc->adap[j] = *((struct i2c_adapter *)platform_get_drvdata(soc->xlink_i2c_plt_dev[j]));
		printk(KERN_INFO "Adapter number = %d\n", soc->adap[j].nr);
	}
	#ifndef CONFIG_HDDL_LOCAL_HOST
	soc->soc_xlinki2c_cnt = sizeof(host_xlinki2c_devices)/sizeof(struct i2c_board_info);

	soc->soc_smbus = kzalloc(sizeof(struct i2c_client *) * (kmb_hddls[board_id].soc[kmb_id].soc_xlinki2c_cnt), GFP_KERNEL);

	soc->soc_xlinki2c = kzalloc(sizeof(struct i2c_client *) * (kmb_hddls[board_id].soc[kmb_id].soc_xlinki2c_cnt), GFP_KERNEL);

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
				soc->soc_smbus[i] = i2c_new_client_device(temp, &temp_host_i2c_device);
				printk("soc_smbus adapter %d", soc->soc_smbus[i]->adapter->nr);
				break;
			}
			j++;
		}
		soc->soc_xlinki2c[i] = i2c_new_client_device(i2c_get_adapter(
				soc->adap[1].nr),
				&temp_host_i2c_device);
		printk("soc_xlinki2c adapter %d", soc->soc_xlinki2c[i]->adapter->nr);

		printk(KERN_INFO
			"Host_kmb_tj [%s] %d\n",
			temp_host_i2c_device.type,
			temp_host_i2c_device.addr);
	}

	if (one_dev == 0) {
		soc->i2c_slaves_cnt = sizeof(soc_i2c_ext_devices)/sizeof(struct i2c_board_info);

		soc->i2c_slaves_ext = kzalloc(sizeof(struct i2c_client *) * (soc->i2c_slaves_cnt), GFP_KERNEL);

		mutex_lock(&my_mutex);

		for (i = 0; i < soc->i2c_slaves_cnt; i++) {
			soc->i2c_slaves_ext[i] = i2c_new_client_device(
				i2c_get_adapter(soc->adap[0].nr),
				&soc_i2c_ext_devices[i]);
			printk("i2c_slaves_ext adapter %d", soc->i2c_slaves_ext[i]->adapter->nr);

			ssleep(1);
		}
		mutex_unlock(&my_mutex);
	}

	#else
	soc->soc_xlinki2c_cnt = sizeof(kmb_i2c_devices)/sizeof(struct i2c_board_info);
	soc->soc_xlinki2c = kzalloc(sizeof(struct i2c_client *) * (soc->soc_xlinki2c_cnt), GFP_KERNEL);
	for (i = 0; i < soc->soc_xlinki2c_cnt; i++) {
		soc->soc_xlinki2c[i] = i2c_new_client_device(
			i2c_get_adapter(soc->adap[1].nr),
			&kmb_i2c_devices[i]);
	}
	#endif
	return 0;
}





#define SW_DEVICE_ID_INTERFACE_SHIFT 24U
#define SW_DEVICE_ID_INTERFACE_MASK  0x7
#define GET_INTERFACE_FROM_SW_DEVICE_ID(id) \
		((id >> SW_DEVICE_ID_INTERFACE_SHIFT) & SW_DEVICE_ID_INTERFACE_MASK)
#define SW_DEVICE_ID_IPC_INTERFACE  0x0
#define SW_DEVICE_ID_PCIE_INTERFACE 0x1

#define XLINK_CONNECT_THREAD_BASED
static struct xlink_handle xdevH[20];
static int hddl_device_thermal_init(void *thread_param)
{
	uint32_t num_devices = 0;
	uint32_t sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t i = 0;

	printk(KERN_INFO "HDDL:Getting device list...\n");
#ifndef CONFIG_HDDL_LOCAL_HOST
	mutex_init(&my_mutex);
#endif

	xlink_get_device_list(sw_device_id_list, &num_devices);
	if (num_devices == 0) {
		printk(KERN_INFO "HDDL:No devices found\n");
		return 0;
	} else if (num_devices > 20) {
		printk(KERN_ERR "HDDL:Increase xdevH array size\n");
		return -1;
	}
	printk(KERN_INFO "HDDL:%u devices found...\n", num_devices);
	for (i = 0; i < num_devices; i++) {
		memset(&xdevH[i], 0, sizeof(struct xlink_handle));

		xdevH[i].dev_type = HOST_DEVICE;
		xdevH[i].sw_device_id = sw_device_id_list[i];
		xlink_get_device_name(&(xdevH[i]),
		device_name, XLINK_MAX_DEVICE_NAME_SIZE);
		printk(KERN_INFO "HDDL:Device name: %x %s\n",
		xdevH[i].sw_device_id, device_name);
	}

	for (i = 0; i < num_devices; i++) {
		struct task_struct *task_recv;
		if (GET_INTERFACE_FROM_SW_DEVICE_ID(xdevH[i].sw_device_id) == SW_DEVICE_ID_PCIE_INTERFACE) {
			printk(KERN_INFO "HDDL:Booting device...\n");
			task_recv = kthread_run(hddl_per_device_connect_thread,
			(void *)&xdevH[i],
			"hddl_per_device_thread");
			if (task_recv == NULL)
				printk(KERN_WARNING "hddl_device_init Thread creation failed");
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
	printk(KERN_INFO "Device Driver Remove...Done!!!\n");

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
	if (task_recv == NULL) {
		printk(KERN_WARNING
		"hddl_device_init Thread creation failed");
	}
		/*ioctl*/

  /*Allocating Major number*/
	  if ((alloc_chrdev_region(&dev, 0, 1, "Hddl_Dev")) < 0) {
		printk(KERN_INFO "Cannot allocate major number\n");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));
	/*Creating cdev structure*/
	cdev_init(&hddl_cdev, &fops);
	/*Adding character device to the system*/
	if ((cdev_add(&hddl_cdev, dev, 1)) < 0) {
		printk(KERN_INFO "Cannot add the device to the system\n");
		goto r_class;
	}
	/*Creating struct class*/
	dev_class = class_create(THIS_MODULE, "hddl_class");
	if (dev_class == NULL) {
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/*Creating device*/
	if ((device_create(dev_class, NULL, dev, NULL, "hddl_device")) == NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}
	printk(KERN_INFO "Device Driver Insert...Done!!!\n");
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
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai"
	      "<lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_LICENSE("GPL v2");
