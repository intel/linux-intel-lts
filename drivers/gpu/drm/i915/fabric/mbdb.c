// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2023 Intel Corporation.
 *
 */

#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/io.h>

#include "fw.h"
#include "iaf_drv.h"
#include "io.h"
#include "mbdb.h"
#include "ops.h"
#include "port.h"
#include "trace.h"

/* Mailbox private structures */

#define CP_ADDR_MBDB_INT_STATUS_UNMASKED (CP_ADDR_MBDB_BASE + 0x00)
#define CP_ADDR_MBDB_INT_ENABLE_MASK     (CP_ADDR_MBDB_BASE + 0x08)
#define CP_ADDR_MBDB_INT_STATUS_MASKED   (CP_ADDR_MBDB_BASE + 0x10)
#define CP_ADDR_MBDB_INT_ACK             (CP_ADDR_MBDB_BASE + 0x18)
#define CP_ADDR_MBDB_INT_PARTNER_SET     (CP_ADDR_MBDB_BASE + 0x20)
#define CP_ADDR_MBDB_INT_STATUS_GPSB     (CP_ADDR_MBDB_BASE + 0x28)
#define CP_ADDR_MBDB_DB0_STATUS          (CP_ADDR_MBDB_BASE + 0x40)
#define CP_ADDR_MBDB_DB0_DEC             (CP_ADDR_MBDB_BASE + 0x48)
#define CP_ADDR_MBDB_DB0_RESET           (CP_ADDR_MBDB_BASE + 0x50)
#define CP_ADDR_MBDB_DB1_STATUS          (CP_ADDR_MBDB_BASE + 0x60)
#define CP_ADDR_MBDB_DB1_DEC             (CP_ADDR_MBDB_BASE + 0x68)
#define CP_ADDR_MBDB_DB1_RESET           (CP_ADDR_MBDB_BASE + 0x60)
#define CP_ADDR_MBDB_DB2_STATUS          (CP_ADDR_MBDB_BASE + 0x80)
#define CP_ADDR_MBDB_DB2_INC             (CP_ADDR_MBDB_BASE + 0x88)
#define CP_ADDR_MBDB_DB3_STATUS          (CP_ADDR_MBDB_BASE + 0xA0)
#define CP_ADDR_MBDB_DB3_INC             (CP_ADDR_MBDB_BASE + 0xA8)
#define CP_ADDR_MBDB_SEM0_ACQUIRE        (CP_ADDR_MBDB_BASE + 0xC0)
#define CP_ADDR_MBDB_SEM0_RELEASE        (CP_ADDR_MBDB_BASE + 0xC8)
#define CP_ADDR_MBDB_SEM1_ACQUIRE        (CP_ADDR_MBDB_BASE + 0xD0)
#define CP_ADDR_MBDB_SEM1_RELEASE        (CP_ADDR_MBDB_BASE + 0xD8)
#define CP_ADDR_MBDB_SEM2_ACQUIRE        (CP_ADDR_MBDB_BASE + 0xE0)
#define CP_ADDR_MBDB_SEM2_RELEASE        (CP_ADDR_MBDB_BASE + 0xE8)
#define CP_ADDR_MBDB_SEM3_ACQUIRE        (CP_ADDR_MBDB_BASE + 0xF0)
#define CP_ADDR_MBDB_SEM3_RELEASE        (CP_ADDR_MBDB_BASE + 0xF8)
#define CP_ADDR_MBDB_MISC_SHARED         (CP_ADDR_MBDB_BASE + 0x100)
#define CP_ADDR_MBDB_GP_DATA             (CP_ADDR_MBDB_BASE + 0x1000)

#define S_INT_STATUS_DB0_INT       0
#define S_INT_STATUS_DB1_INT       1
#define S_INT_STATUS_SEM0_ACQUIRED 2
#define S_INT_STATUS_SEM1_ACQUIRED 3
#define S_INT_STATUS_SEM2_ACQUIRED 4
#define S_INT_STATUS_SEM3_ACQUIRED 5
#define S_INT_STATUS_INBOX_FULL    6
#define S_INT_STATUS_OUTBOX_EMPTY  7

#define INBOX_FULL_MASK   (BIT_ULL_MASK(S_INT_STATUS_INBOX_FULL))
#define OUTBOX_EMPTY_MASK (BIT_ULL_MASK(S_INT_STATUS_OUTBOX_EMPTY))

#define TIMEOUT (60 * HZ)

#include <linux/module.h>
#include <linux/timer.h>

struct mbdb {
	struct fsubdev *sd;
	bool stopping;
	bool polling;
	atomic_t pending_new_seq;

	u64 __iomem *int_status_unmasked_addr;
	u64 __iomem *int_status_masked_addr;
	u64 __iomem *int_ack_addr;
	u64 __iomem *int_partner_set_addr;
	u64 __iomem *int_enable_mask_addr;
	u64 __iomem *gp_outbox_data_addr;
	u64 __iomem *gp_outbox_cw_addr;
	u64 __iomem *gp_outbox_param_addr;
	u64 __iomem *gp_inbox_data_addr;
	u64 __iomem *gp_inbox_cw_addr;
	u64 __iomem *gp_inbox_param_addr;

	u64 __iomem *gp_misc_shared;

	/* Protect outbox access */
	struct semaphore outbox_sem;
	struct completion outbox_empty;
	u8 outbox_seqno;

	struct work_struct inbox_full;
	u8 inbox_seqno;

	/* tid get+increment protected by inbox_mutex */
	u32 inbox_tid;

	/* list mutation protected by inbox_mutex */
	struct list_head ibox_list;

	/* Protect irq access */
	spinlock_t irq_spinlock;
	u64 int_enables;

	/* Protect pci bar space access */
	struct mutex ack_mutex;
	/* Protect pci bar space access */
	struct mutex partner_set_mutex;

	u64 counters[MBDB_COUNTERS_MAX];

	/* Serializes inbox accesses */
	struct mutex inbox_mutex;
	atomic_t ibox_waiters;
	struct completion ibox_waiters_done;

	/* Protect inbox access */
	struct timer_list inbox_timer;
};

#define MBDB_SLOW_POLL_TIMEOUT (HZ)
#define MBDB_FAST_POLL_TIMEOUT 5

static unsigned int mbdb_polling_mode;

static void mbdb_disable_interrupts(struct mbdb *mbdb, u64 intr_mask);
static void mbdb_enable_interrupts(struct mbdb *mbdb, u64 intr_mask);

static void inbox_full_enqueue(struct mbdb *mbdb)
{
	/*
	 * general-purpose guard to ensure that a post-stop flush() in the
	 * destroy path is sufficient to guarantee the inbox worker is dead
	 */
	if (!READ_ONCE(mbdb->stopping))
		queue_work(iaf_unbound_wq, &mbdb->inbox_full);
}

static void inbox_timer_fn(struct timer_list *timer)
{
	struct mbdb *mbdb = from_timer(mbdb, timer, inbox_timer);

	inbox_full_enqueue(mbdb);
	mod_timer(&mbdb->inbox_timer, jiffies + MBDB_SLOW_POLL_TIMEOUT);
}

/**
 * mbdb_readq - Perform csr readq checking for pci bus in error condition
 * @mbdb: mailbox being checked
 * @addr: address of csr to read
 * @reg_val: address where read csr data should be returned
 *
 * This should only be used for status csr reads as they never return ~0ULL.
 * Data reads of the mailbox, in response to a mailbox operation, can return ~0ULL and should NEVER
 * call this function as this would cause the device to be disabled when it is functioning normally.
 *
 * Return:
 * * 0 on success
 * * -EIO on failure
 */
static int mbdb_readq(struct mbdb *mbdb, u64 __iomem *addr, u64 *reg_val)
{
	if (unlikely(READ_ONCE(mbdb->sd->fdev->dev_disabled)))
		return -EIO;

	*reg_val = readq(addr);

	/* Check if the pci bus has failed */
	if (unlikely(*reg_val == ~0ULL)) {
		WRITE_ONCE(mbdb->sd->fdev->dev_disabled, true);
		dev_err(fdev_dev(mbdb->sd->fdev),
			"PCIE bus error detected. Device has been disabled\n");
		return -EIO;
	}

	return 0;
}

static int mbdb_wait_outbox_empty(struct mbdb *mbdb)
{
	int wait_time;
	int timeout;
	u64 reg_val;

	for (wait_time = 0; wait_time < TIMEOUT;
	     wait_time += MBDB_FAST_POLL_TIMEOUT - timeout) {
		if (READ_ONCE(mbdb->stopping))
			return -ENODEV;

		if (mbdb_readq(mbdb, mbdb->int_status_unmasked_addr, &reg_val))
			return -EIO;

		if (reg_val & OUTBOX_EMPTY_MASK)
			return 0;

		timeout =
			schedule_timeout_interruptible(MBDB_FAST_POLL_TIMEOUT);
	}

	return -ETIMEDOUT;
}

#define MBDB_POLLING_MODE_TILE_ENABLE BIT(31)
#define MBDB_POLLING_MODE_TILE_MASK GENMASK(IAF_MAX_SUB_DEVS - 1, 0)

static int mbdb_polling_mode_set(const char *val, const struct kernel_param *kp)
{
	unsigned int as_uint;
	bool as_bool;
	bool is_uint;
	bool is_bool;

	is_uint = !kstrtouint(val, 0, &as_uint);
	is_bool = !strtobool(val, &as_bool);

	/*
	 * check for boolean mode: interpret as a single boolean that globally
	 * selects interrupts (off) or polling (on), applying WAs as necessary.
	 */
	if (is_uint && as_uint <= 1) {
		mbdb_polling_mode = as_uint;
		return 0;
	}
	if (!is_uint && is_bool) {
		mbdb_polling_mode = as_bool;
		return 0;
	}

	/*
	 * check for tile mask mode: interpret the lower bits as explicit
	 * polling mode enables for each corresponding tile, with WAs disabled
	 */
	if (is_uint) {
		if (!(as_uint & MBDB_POLLING_MODE_TILE_ENABLE)) {
			/* the mask bit MUST be set */
			return -EINVAL;
		}
		mbdb_polling_mode = as_uint;
		return 0;
	}

	return -EINVAL;
}

static const struct kernel_param_ops mbdb_polling_mode_ops = {
	.set = mbdb_polling_mode_set,
	.get = param_get_uint,
};

/*
 * If bit 31 is clear, the remaining bits are interpreted as a boolean value
 * that enables polling mode as documented in the help text.  In this mode, any
 * workarounds relevant to the device will be applied.  Intended as a general
 * debug workaround for interrupt issues.
 *
 * If bit 31 is set, the remaining bits are interpreted as a mask of tiles
 * to explicitly set polling mode for.  In this mode, workarounds are NOT
 * applied and the user is in complete control of the mailbox mode.  Intended
 * for debugging software in the mailbox control path that depends on a
 * particular mode.
 */
module_param_cb(mbdb_polling_mode, &mbdb_polling_mode_ops, &mbdb_polling_mode, 0400);
MODULE_PARM_DESC(mbdb_polling_mode,
		 "Use polling instead of interrupts to access Mailbox (default: N)");

/**
 * mbdb_apply_pvc_rc6_wa - use polling mode on tile 1 to avoid
 * RC6-related mailbox latency (WA_14017048588).
 * @mbdb: subdevice mailbox reference
 */
static void mbdb_apply_pvc_rc6_wa(struct mbdb *mbdb)
{
	struct fsubdev *sd = mbdb->sd;

	if (mbdb->polling)
		return;

	if (sd->fdev->pd->product == IAF_PONTEVECCHIO && sd_index(sd) == 1) {
		sd_info(sd, "mbdb polling mode pvc wa enabled for tile 1\n");
		mbdb->polling = true;
	}
}

static void mbdb_polling_mode_apply(struct mbdb *mbdb)
{
	if (mbdb_polling_mode & MBDB_POLLING_MODE_TILE_ENABLE) {
		/* do NOT apply any WAs in tile mask mode */
		mbdb->polling = mbdb_polling_mode
			      & MBDB_POLLING_MODE_TILE_MASK
			      & BIT(sd_index(mbdb->sd));
	} else {
		/* apply applicable WAs in legacy boolean mode */
		mbdb->polling = mbdb_polling_mode;
		mbdb_apply_pvc_rc6_wa(mbdb);
	}
}

/**
 * mbdb_init_module - Early device-independent module initialization
 */
void mbdb_init_module(void)
{
	unsigned int tile, mask;

	if (!mbdb_polling_mode) {
		pr_debug("mailbox mode: interrupts enabled\n");
		return;
	}

	if (!(mbdb_polling_mode & MBDB_POLLING_MODE_TILE_ENABLE)) {
		pr_info("mailbox mode: polling enabled for all tiles\n");
		return;
	}

	pr_info("mailbox mode: workarounds disabled\n");
	for (tile = 0, mask = 1; mask < MBDB_POLLING_MODE_TILE_MASK; ++tile, mask <<= 1) {
		if (mbdb_polling_mode & mask)
			pr_info("mailbox mode: polling enabled for tile %u\n", tile);
	}
}

static void mbdb_int_ack_wr(struct mbdb *mbdb, u64 mask)
{
	if (unlikely(READ_ONCE(mbdb->sd->fdev->dev_disabled)))
		return;

	mutex_lock(&mbdb->ack_mutex);
	writeq(mask, mbdb->int_ack_addr);
	mutex_unlock(&mbdb->ack_mutex);
}

static void mbdb_int_partner_set_wr(struct mbdb *mbdb, u64 mask)
{
	if (unlikely(READ_ONCE(mbdb->sd->fdev->dev_disabled)))
		return;

	mutex_lock(&mbdb->partner_set_mutex);
	writeq(mask, mbdb->int_partner_set_addr);
	mutex_unlock(&mbdb->partner_set_mutex);
}

static void mbdb_enable_interrupts(struct mbdb *mbdb, u64 intr_mask)
{
	unsigned long flags;

	if (READ_ONCE(mbdb->stopping))
		return;

	if (unlikely(READ_ONCE(mbdb->sd->fdev->dev_disabled)))
		return;

	spin_lock_irqsave(&mbdb->irq_spinlock, flags);

	if (~mbdb->int_enables & intr_mask) {
		mbdb->int_enables |= intr_mask;
		writeq(mbdb->int_enables, mbdb->int_enable_mask_addr);
	}

	spin_unlock_irqrestore(&mbdb->irq_spinlock, flags);
}

static void mbdb_disable_interrupts(struct mbdb *mbdb, u64 intr_mask)
{
	unsigned long flags;

	if (unlikely(READ_ONCE(mbdb->sd->fdev->dev_disabled)))
		return;

	spin_lock_irqsave(&mbdb->irq_spinlock, flags);

	if (mbdb->int_enables & intr_mask) {
		mbdb->int_enables &= ~intr_mask;
		writeq(mbdb->int_enables, mbdb->int_enable_mask_addr);
	}

	spin_unlock_irqrestore(&mbdb->irq_spinlock, flags);
}

irqreturn_t mbdb_handle_irq(struct fsubdev *sd)
{
	struct mbdb *mbdb = sd->mbdb;
	u64 contrib;

	if (!mbdb)
		return IRQ_NONE;

	if (mbdb_readq(mbdb, mbdb->int_status_masked_addr, &contrib))
		return IRQ_HANDLED;

	/*
	 * This warrants some further investigation: If this indicates an
	 * interrupt for THIS IAF, returning IRQ_HANDLED might be more
	 * resilient, though that probably should never happen anyway.
	 */
	if (!contrib)
		return IRQ_NONE;

	mbdb_disable_interrupts(mbdb, contrib);

	if (contrib & INBOX_FULL_MASK)
		inbox_full_enqueue(mbdb);

	if (contrib & OUTBOX_EMPTY_MASK)
		complete(&mbdb->outbox_empty);

	return IRQ_HANDLED;
}

u64 mbdb_get_mbox_comm_errors(struct fsubdev *sd)
{
	u64 *counters = sd->mbdb->counters;

	return counters[TIMEDOUT_REQUESTS] + counters[UNHANDLED_RECEIVED_REQUESTS] +
	       counters[ERROR_RESPONSES] + counters[UNMATCHED_RESPONSES] +
	       counters[TIMEDOUT_RESPONSES];
}

u64 *mbdb_get_mailbox_counters(struct fsubdev *sd)
{
	return sd->mbdb->counters;
}

/**
 * mbdb_stop_from_handler - Stops mailbox processing without tearing down the
 * mailbox
 * @mbdb: mailbox reference
 *
 * Expects to be called from the inbox handler (i.e. with the inbox mutex
 * held and interrupts disabled) when it encounters a fatal condition.
 *
 * This just marks the mailbox as stopping to prevent any future operations
 * from making progress, disables the timer to prevent mailbox processing, and
 * pre-emptively wakes any inbox/outbox waiters, which will abandon their
 * operation upon waking since stopping is true.  If they're already awake and
 * post-wait, they'll continue as normal since we're not invalidating any
 * resources here.
 */
static void mbdb_stop_from_handler(struct mbdb *mbdb)
{
	struct mbdb_ibox *ibox;

	lockdep_assert_held(&mbdb->inbox_mutex);

	WRITE_ONCE(mbdb->stopping, true);

	if (mbdb->polling)
		del_timer_sync(&mbdb->inbox_timer);
	else
		complete(&mbdb->outbox_empty);

	list_for_each_entry(ibox, &mbdb->ibox_list, ibox_list_link)
		complete(&ibox->ibox_full);
}

static void mbdb_inbox_empty(struct mbdb *mbdb)
{
	/* tell our partner it can put a new message in its outbox */

	if (!mbdb->polling)
		mbdb_enable_interrupts(mbdb, INBOX_FULL_MASK);

	mbdb_int_partner_set_wr(mbdb, OUTBOX_EMPTY_MASK);

	if (mbdb->polling)
		inbox_full_enqueue(mbdb);
}

static void mbdb_ibox_handle_request(struct fsubdev *sd, u64 cw)
{
	u16 len;

	switch (mbdb_mbox_op_code(cw)) {
	case MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_NOTIFICATION:
		sd->mbdb->counters[HANDLED_RECEIVED_REQUESTS]++;
		port_state_change_trap_handler(sd);
		break;

	case MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_NOTIFICATION:
		sd->mbdb->counters[HANDLED_RECEIVED_REQUESTS]++;
		port_link_width_degrade_trap_handler(sd);
		break;

	case MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_NOTIFICATION:
		sd->mbdb->counters[HANDLED_RECEIVED_REQUESTS]++;
		port_link_quality_indicator_trap_handler(sd);
		break;

	case MBOX_OP_CODE_QSFP_MGR_FAULT_TRAP_NOTIFICATION:
		sd->mbdb->counters[HANDLED_RECEIVED_REQUESTS]++;
		port_qsfp_fault_trap_handler(sd);
		break;

	/*
	 * Not implemented yet:
	 *
	 * case MBOX_OP_CODE_QSFP_MGR_PRESENCE_TRAP_NOTIFICATION:
	 *	sd-mbdb->counters[HANDLED_REQUESTS]++;
	 *	port_qsfp_presence_trap_handler(sd);
	 *	break;
	 */

	case MBOX_OP_CODE_FAULT_TRAP:
		sd->mbdb->counters[HANDLED_RECEIVED_REQUESTS]++;
		mbdb_stop_from_handler(sd->mbdb);
		sd_err(sd, "firmware reported fatal error: cause 0x%08x\n", mbdb_mbox_tid(cw));
		len = mbdb_mbox_params_len(cw);
		if (len) {
			u64 __iomem *addr = sd->mbdb->gp_inbox_param_addr;
			char optional[16];

			if (len > MBOX_PARAM_AREA_IN_BYTES)
				len = MBOX_PARAM_AREA_IN_BYTES;
			while (len) {
				char line[80];
				u16 chunk = (len < sizeof(optional)) ?
					    len : sizeof(optional);

				if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
					return;

				io_readq_aligned(addr, optional, chunk);
				hex_dump_to_buffer(optional, chunk,
						   sizeof(optional), 1, line,
						   sizeof(line), true);
				sd_err(sd, "%s", line);
				len -= chunk;
				addr += chunk / sizeof(u64);
			}
		}
		indicate_subdevice_error(sd, SD_ERROR_FW);
		break;
	default:
		sd->mbdb->counters[UNHANDLED_RECEIVED_REQUESTS]++;
		sd_warn(sd, "Received MBOX request cw=0x%016llx opCode=%u tid=%u rsp_status=%u\n",
			cw, mbdb_mbox_op_code(cw), mbdb_mbox_tid(cw), mbdb_mbox_rsp_status(cw));
		break;
	}
}

/**
 * mbdb_seq_reset - Resets sequence numbers on a successful bootloader<->fw
 * transition, and unblocks the outbox.
 * @ibox: inbox to operate on
 * @cw: inbound control word in flight
 */
static void mbdb_seq_reset(struct mbdb_ibox *ibox, u64 cw)
{
	struct mbdb *mbdb = ibox->mbdb;

	/* This only applies to FW_START and RESET responses */
	if (mbdb_mbox_op_code(cw) != MBOX_OP_CODE_RESET &&
	    mbdb_mbox_op_code(cw) != MBOX_OP_CODE_FW_START)
		return;

	if (mbdb_mbox_rsp_status(cw) != MBOX_RSP_STATUS_OK) {
		/* don't reset seqs if the ops were rejected */
	} else if (mbdb_mbox_op_code(cw) == MBOX_OP_CODE_FW_START) {
		/*
		 * FW_START encodes the result of the start attempt as a U32 in
		 * the response payload.  only reset seqs if this succeeded
		 */
		u32 result = *(u32 *)ibox->response;

		if (!result) {
			mbdb->inbox_seqno = 0;
			mbdb->outbox_seqno = 0;
		}
	} else {
		/* RESET has no payload */
		mbdb->inbox_seqno = 0;
		mbdb->outbox_seqno = 0;
	}

	/*
	 * whether pass or fail, we're no longer pending a new seq and can
	 * release the outbox to other users
	 */
	if (atomic_cmpxchg(&mbdb->pending_new_seq, 1, 0))
		up(&mbdb->outbox_sem);
}

static struct mbdb_ibox *mbdb_find_ibox(struct mbdb *mbdb, u32 tid, u8 op_code)
{
	struct mbdb_ibox *ibox;
	bool found = false;

	lockdep_assert_held(&mbdb->inbox_mutex);

	/*
	 * see if we can find the associated ibox in the list for this
	 * op_code and tid
	 */

	list_for_each_entry(ibox, &mbdb->ibox_list, ibox_list_link) {
		if (ibox->tid == tid && ibox->op_code == op_code) {
			found = true;
			break;
		}
	}

	return found ? ibox : NULL;
}

static void mbdb_update_response_counters(struct mbdb *mbdb, u8 rsp_status, u64 cw)
{
	switch (rsp_status) {
	case MBOX_RSP_STATUS_OK:
		mbdb->counters[NON_ERROR_RESPONSES]++;
		break;
	case MBOX_RSP_STATUS_RETRY:
		mbdb->counters[RETRY_RESPONSES]++;
		break;
	case MBOX_RSP_STATUS_SEQ_NO_ERROR:
		mbdb->counters[OUTBOUND_SEQNUM_MISMATCHES]++;
		sd_warn(mbdb->sd, "Synchronizing outbound sequence number\n");
		break;
	case MBOX_RSP_STATUS_OP_CODE_ERROR:
	case MBOX_RSP_STATUS_LOGICAL_ERROR:
	case MBOX_RSP_STATUS_DENIED:
		mbdb->counters[ERROR_RESPONSES]++;
		sd_err(mbdb->sd, "Errored MBOX control word: 0x%016llx\n", cw);
		break;
	default:
		mbdb->counters[ERROR_RESPONSES]++;
		break;
	}
}

static void mbdb_ibox_handle_response(struct mbdb *mbdb, u64 cw)
{
	struct mbdb_ibox *ibox = mbdb_find_ibox(mbdb, mbdb_mbox_tid(cw), mbdb_mbox_op_code(cw));

	if (likely(ibox)) {
		if (mbdb_mbox_params_len(cw) > MBOX_PARAM_AREA_IN_BYTES) {
			ibox->rsp_status = MBOX_RSP_STATUS_LOGICAL_ERROR;
		} else {
			if (unlikely(READ_ONCE(mbdb->sd->fdev->dev_disabled))) {
				ibox->rsp_status = -EIO;
			} else {
				ibox->cw = cw;
				ibox->rsp_status = mbdb_mbox_rsp_status(cw);

				if (ibox->rsp_status == MBOX_RSP_STATUS_OK)
					/* transfer the h/w data into the virtual ibox */
					ibox->op_rsp_handler(ibox);
			}
		}

		mbdb_update_response_counters(mbdb, ibox->rsp_status, cw);

		mbdb_seq_reset(ibox, cw);

		complete(&ibox->ibox_full);
	} else {
		mbdb->counters[UNMATCHED_RESPONSES]++;
		if (mbdb_mbox_is_posted(cw))
			sd_warn(mbdb->sd, "Received response to posted request cw=0x%016llx op_code=%u tid=%u rsp_status=%u\n",
				cw, mbdb_mbox_op_code(cw), mbdb_mbox_tid(cw),
				mbdb_mbox_rsp_status(cw));
		else
			sd_warn(mbdb->sd, "NOT found cw=0x%016llx op_code=%u tid=%u rsp_status=%u\n",
				cw, mbdb_mbox_op_code(cw), mbdb_mbox_tid(cw),
				mbdb_mbox_rsp_status(cw));
	}
}

static bool mbdb_get_polling_input(struct mbdb *mbdb)
{
	u64 int_status_unmasked;

	if (mbdb_readq(mbdb, mbdb->int_status_unmasked_addr, &int_status_unmasked))
		return false;

	if (!(int_status_unmasked & INBOX_FULL_MASK)) {
		if (atomic_read(&mbdb->ibox_waiters))
			inbox_full_enqueue(mbdb);
		return false;
	}

	return true;
}

static void mbdb_inbox_full_fn(struct work_struct *inbox_full)
{
	struct mbdb *mbdb = container_of(inbox_full, struct mbdb, inbox_full);
	u64 cw;

	if (READ_ONCE(mbdb->stopping))
		return;

	mutex_lock(&mbdb->inbox_mutex);

	if (mbdb->polling)
		if (!mbdb_get_polling_input(mbdb))
			goto exit;

	/* Get the control word info */
	if (mbdb_readq(mbdb, mbdb->gp_inbox_cw_addr, &cw))
		goto exit;

	/* acknowlege the inbox full */
	mbdb_int_ack_wr(mbdb, INBOX_FULL_MASK);

	trace_iaf_mbx_in(mbdb->sd->fdev->pd->index, sd_index(mbdb->sd), mbdb_mbox_op_code(cw),
			 mbdb->inbox_seqno, mbdb_mbox_seq_no(cw), mbdb_mbox_rsp_status(cw),
			 mbdb_mbox_tid(cw), cw);

	/* Check that the "inbound" sequence number in the cw matches what we expect to receive */
	if (mbdb_mbox_seqno_error(mbdb_mbox_seq_no(cw), mbdb->inbox_seqno)) {
		mbdb->counters[INBOUND_SEQNUM_MISMATCHES]++;
		sd_warn(mbdb->sd, "Synchronizing inbound sequence number\n");
	}

	/*
	 * synchronize our expected "inbound" sequence number for the next message received from the
	 * firmware
	 */
	mbdb->inbox_seqno = mbdb_mbox_seqno_next(mbdb_mbox_seq_no(cw));

	if (mbdb_mbox_msg_type(cw) == MBOX_REQUEST)
		mbdb_ibox_handle_request(mbdb->sd, cw);
	else
		mbdb_ibox_handle_response(mbdb, cw);

	mbdb_inbox_empty(mbdb);

exit:
	mutex_unlock(&mbdb->inbox_mutex);
}

static void mbdb_init(struct mbdb *mbdb)
{
	mbdb->int_enables = 0ull;
	mbdb_int_partner_set_wr(mbdb, OUTBOX_EMPTY_MASK);

	if (!mbdb->polling)
		mbdb_enable_interrupts(mbdb, INBOX_FULL_MASK);
}

struct fsubdev *mbdb_ibox_sd(struct mbdb_ibox *ibox)
{
	return ibox->mbdb->sd;
}

u64 __iomem *mbdb_ibox_gp_inbox_param_addr(struct mbdb_ibox *ibox)
{
	return ibox->mbdb->gp_inbox_param_addr;
}

void mbdb_reinit(struct fsubdev *sd)
{
	if (sd && sd->mbdb)
		mbdb_init(sd->mbdb);
}

u8 mbdb_outbox_seqno(struct fsubdev *sd)
{
	u8 result;

	result = sd->mbdb->outbox_seqno;
	sd->mbdb->outbox_seqno = mbdb_mbox_seqno_next(result);
	return result;
}

static int mbdb_outbox_is_empty(struct mbdb *mbdb)
{
	bool completed;
	u64 reg_val;

	if (mbdb_readq(mbdb, mbdb->int_status_unmasked_addr, &reg_val))
		return -EIO;

	if (reg_val & OUTBOX_EMPTY_MASK)
		return 0;

	if (mbdb->polling) {
		int ret = mbdb_wait_outbox_empty(mbdb);

		if (ret == -ETIMEDOUT) {
			mbdb->counters[TIMEDOUT_REQUESTS]++;
			sd_err(mbdb->sd, "OUTBOX timed out\n");
		}

		return ret;
	}

	mbdb_enable_interrupts(mbdb, OUTBOX_EMPTY_MASK);

	completed = wait_for_completion_timeout(&mbdb->outbox_empty, TIMEOUT);
	if (READ_ONCE(mbdb->stopping))
		return -ENODEV;

	if (completed) {
		reinit_completion(&mbdb->outbox_empty);
		return 0;
	}

	if (mbdb_readq(mbdb, mbdb->int_status_unmasked_addr, &reg_val))
		return -EIO;

	if (reg_val & OUTBOX_EMPTY_MASK)
		return 0;

	mbdb->counters[TIMEDOUT_REQUESTS]++;
	sd_err(mbdb->sd, "OUTBOX timed out\n");

	return -ETIMEDOUT;
}

struct mbox_msg __iomem *mbdb_outbox_acquire(struct fsubdev *sd, u64 *cw)
{
	struct mbdb *mbdb = sd->mbdb;
	int ret;

	if  (!mbdb)
		return IOMEM_ERR_PTR(-EINVAL);

	ret = down_killable(&mbdb->outbox_sem);
	if (ret)
		return IOMEM_ERR_PTR(ret);

	if (READ_ONCE(mbdb->stopping)) {
		up(&mbdb->outbox_sem);
		return IOMEM_ERR_PTR(-ENODEV);
	}

	if (mbdb_mbox_is_posted(*cw) == MBOX_NO_RESPONSE_REQUESTED)
		++mbdb->counters[POSTED_REQUESTS];
	else
		++mbdb->counters[NON_POSTED_REQUESTS];

	ret = mbdb_outbox_is_empty(mbdb);
	if (ret < 0) {
		up(&mbdb->outbox_sem);
		return IOMEM_ERR_PTR(ret);
	}

	mbdb_int_ack_wr(mbdb, OUTBOX_EMPTY_MASK);

	/* Add a sequence number to the control word */
	*cw |= FIELD_PREP(MBOX_CW_SEQ_NO, mbdb_outbox_seqno(sd));

	/* For posted RESET, reset sequences immediately */
	if (mbdb_mbox_op_code(*cw) == MBOX_OP_CODE_RESET &&
	    mbdb_mbox_is_posted(*cw) == MBOX_NO_RESPONSE_REQUESTED) {
		mbdb->inbox_seqno = 0;
		mbdb->outbox_seqno = 0;
	}

	/* For FW_START and posted RESET, lock outbox */
	if (mbdb_mbox_op_code(*cw) == MBOX_OP_CODE_FW_START ||
	    (mbdb_mbox_op_code(*cw) == MBOX_OP_CODE_RESET &&
	     mbdb_mbox_is_posted(*cw) == MBOX_RESPONSE_REQUESTED))
		atomic_set(&mbdb->pending_new_seq, 1);

	return (struct mbox_msg __iomem *)mbdb->gp_outbox_data_addr;
}

/**
 * mbdb_outbox_release - Releases the outbox.
 * @sd: The subdevice to operate on.
 *
 * Will not release the outbox lock if the operation issued was a firmware
 * start.  Instead the outbox will remain blocked until the fw start operation
 * resolves to prevent sequence number desyncs.
 */
void mbdb_outbox_release(struct fsubdev *sd)
{
	struct mbdb *mbdb = sd->mbdb;

	if (!mbdb)
		return;

	mbdb_int_partner_set_wr(mbdb, INBOX_FULL_MASK);

	if (!atomic_read(&mbdb->pending_new_seq))
		up(&mbdb->outbox_sem);
}

/**
 * mbdb_tile_number_set - Help firmware identify which tile it's running on.
 * @sd: The subdevice to operate on.
 *
 * Hardware provides no straps to communicate tile-number to firmware. Since KMD knows ANR's tile
 * number (a.k.a. subdevice), we can communicate it here. This should be called after downloading
 * firmware but before starting it.
 */
void mbdb_tile_number_set(struct fsubdev *sd)
{
	u64 tmp;
	struct mbdb *mbdb = sd->mbdb;

	if (!mbdb)
		return;

	if (mbdb_readq(mbdb, mbdb->gp_misc_shared, &tmp))
		return;

#define MISC_SHARED_TILE_NUM GENMASK(7, 0)

	tmp = u64_replace_bits(tmp, sd_index(sd), MISC_SHARED_TILE_NUM);
	writeq(tmp, mbdb->gp_misc_shared);
}

struct mbdb_ibox *mbdb_ibox_acquire(struct fsubdev *sd, u8 op_code, void *response, u32 rsp_len,
				    bool posted, op_response_handler op_rsp_handler)
{
	struct mbdb *mbdb = sd->mbdb;
	struct mbdb_ibox *ibox;

	if (!mbdb)
		return ERR_PTR(-EINVAL);

	if (posted)
		return NULL;

	ibox = kzalloc(sizeof(*ibox), GFP_KERNEL);
	if (!ibox)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&mbdb->inbox_mutex);

	ibox->mbdb = mbdb;
	ibox->op_code = op_code;

	INIT_LIST_HEAD(&ibox->ibox_list_link);
	ibox->tid = mbdb->inbox_tid++;
	ibox->response = response;
	ibox->rsp_len = response ? rsp_len : 0;
	ibox->op_rsp_handler = op_rsp_handler;

	init_completion(&ibox->ibox_full);

	/*
	 * MBDB shutdown (due to destroy or a fw fault) will mark any inboxes
	 * on the list as complete in order to prevent them from enduring a
	 * full timeout.
	 *
	 * No inboxes should go on the list after that point, and checking for
	 * stopping under the inbox mutex guaratees that it'll be observed to
	 * be true if the MBDB stopping logic has already walked the list.
	 */
	if (READ_ONCE(mbdb->stopping)) {
		mutex_unlock(&mbdb->inbox_mutex);
		kfree(ibox);
		return ERR_PTR(-EINVAL);
	}

	list_add_tail(&ibox->ibox_list_link, &mbdb->ibox_list);

	mutex_unlock(&mbdb->inbox_mutex);

	return ibox;
}

/**
 * mbdb_ibox_release_pre_wait - Remove from the ibox list and free the ibox
 * @ibox: inbox to free
 *
 * Only call this function if mbdb_ibox_wait() has NOT been called on this
 * ibox.
 *
 */
void mbdb_ibox_release_pre_wait(struct mbdb_ibox *ibox)
{
	/* posted requests have no ibox */
	if (!ibox)
		return;

	mutex_lock(&ibox->mbdb->inbox_mutex);
	list_del_init(&ibox->ibox_list_link);
	mutex_unlock(&ibox->mbdb->inbox_mutex);

	kfree(ibox);
}

/**
 * mbdb_ibox_release - Free an ibox that is not on a list
 * @ibox: inbox to free
 *
 */
void mbdb_ibox_release(struct mbdb_ibox *ibox)
{
	kfree(ibox);
}

/**
 * mbdb_ibox_wait - Waits on the specified inbox for IO completion
 * @ibox: inbox to wait on
 *
 * Return: 0 on success, negative errno otherwise.
 */
int mbdb_ibox_wait(struct mbdb_ibox *ibox)
{
	struct mbdb *mbdb = ibox->mbdb;
	bool completed;
	bool stopping;
	int ret = 0;

	atomic_inc(&mbdb->ibox_waiters);

	if (mbdb->polling)
		inbox_full_enqueue(mbdb);

	completed = wait_for_completion_timeout(&ibox->ibox_full, TIMEOUT);

	mutex_lock(&mbdb->inbox_mutex);
	stopping = READ_ONCE(mbdb->stopping);

	/* handle timeout */
	if (!completed && !stopping) {
		/* inbox handler has not run; time the ibox out */
		mbdb->counters[TIMEDOUT_RESPONSES]++;
		sd_err(mbdb->sd, "INBOX timed out: opcode %u tid 0x%08x\n",
		       ibox->op_code, ibox->tid);
		ret = -ETIMEDOUT;

		/*
		 * if we time out a msg that was blocking the outbox for a
		 * seq update, release it
		 */
		if (ibox->op_code == MBOX_OP_CODE_FW_START ||
		    ibox->op_code == MBOX_OP_CODE_RESET)
			if (atomic_cmpxchg(&mbdb->pending_new_seq, 1, 0))
				up(&mbdb->outbox_sem);
	}

	/* ibox processing is complete remove it from the list */
	list_del_init(&ibox->ibox_list_link);
	mutex_unlock(&ibox->mbdb->inbox_mutex);

	/*
	 * the completion is only valid in the mailbox teardown path to
	 * indicate to the inbox drain logic that pending waiters have
	 * completed, so ensure we're stopping before completing it.
	 */
	if (atomic_dec_and_test(&mbdb->ibox_waiters) && stopping)
		complete(&mbdb->ibox_waiters_done);

	if (mbdb->polling)
		inbox_full_enqueue(mbdb);

	return ret;
}

/* assumes mbdb->stopping == true in order for the completions to fire */
static void mbdb_ibox_drain(struct mbdb *mbdb)
{
	struct mbdb_ibox *ibox;

	/* we should not be tearing down the mailbox with live clients */
	if (WARN_ON(!list_empty(&mbdb->ibox_list))) {
		mutex_lock(&mbdb->inbox_mutex);

		/* if waiters remain; force them to time out */
		list_for_each_entry(ibox, &mbdb->ibox_list, ibox_list_link)
			complete(&ibox->ibox_full);

		/* wait for them to complete */
		mutex_unlock(&mbdb->inbox_mutex);

		WARN_ON(wait_for_completion_killable(&mbdb->ibox_waiters_done));
	}
}

void destroy_mbdb(struct fsubdev *sd)
{
	struct mbdb *mbdb = sd->mbdb;

	if (!mbdb)
		return;

	WRITE_ONCE(mbdb->stopping, true);

	if (mbdb->polling) {
		del_timer_sync(&mbdb->inbox_timer);
	} else {
		mbdb_disable_interrupts(mbdb, mbdb->int_enables);
		complete(&mbdb->outbox_empty);
	}

	flush_work(&mbdb->inbox_full);
	mbdb_ibox_drain(mbdb);
	mutex_destroy(&mbdb->partner_set_mutex);
	mutex_destroy(&mbdb->ack_mutex);
	mutex_destroy(&mbdb->inbox_mutex);

	sd->mbdb = NULL;
	kfree(mbdb);
}

static void mbdb_set_mem_addresses(struct mbdb *mbdb)
{
	char __iomem *csr_base = mbdb->sd->csr_base;

	mbdb->int_status_unmasked_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_INT_STATUS_UNMASKED);

	mbdb->int_status_masked_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_INT_STATUS_MASKED);

	mbdb->int_ack_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_INT_ACK);

	mbdb->int_partner_set_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_INT_PARTNER_SET);

	mbdb->int_enable_mask_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_INT_ENABLE_MASK);

	mbdb->gp_outbox_data_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_GP_DATA);

	mbdb->gp_outbox_cw_addr = mbdb->gp_outbox_data_addr;

	mbdb->gp_outbox_param_addr = mbdb->gp_outbox_data_addr + 1;

	mbdb->gp_inbox_data_addr = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_GP_DATA + MBOX_SIZE_IN_BYTES);

	mbdb->gp_inbox_cw_addr = mbdb->gp_inbox_data_addr;

	mbdb->gp_inbox_param_addr = mbdb->gp_inbox_data_addr + 1;

	mbdb->gp_misc_shared = (u64 __iomem *)
		(csr_base + CP_ADDR_MBDB_MISC_SHARED);
}

int create_mbdb(struct fsubdev *sd)
{
	struct mbdb *mbdb;

	mbdb = kzalloc(sizeof(*sd->mbdb), GFP_KERNEL);
	if (!mbdb)
		return -ENOMEM;

	sd->mbdb = mbdb;
	mbdb->sd = sd;
	mbdb->stopping = false;
	atomic_set(&mbdb->pending_new_seq, 0);

	mbdb_polling_mode_apply(mbdb);

	mbdb_set_mem_addresses(mbdb);

	init_completion(&mbdb->outbox_empty);

	INIT_LIST_HEAD(&mbdb->ibox_list);

	INIT_WORK(&mbdb->inbox_full, mbdb_inbox_full_fn);

	sema_init(&mbdb->outbox_sem, 1);

	mutex_init(&mbdb->inbox_mutex);
	mutex_init(&mbdb->ack_mutex);
	mutex_init(&mbdb->partner_set_mutex);

	atomic_set(&mbdb->ibox_waiters, 0);
	init_completion(&mbdb->ibox_waiters_done);

	spin_lock_init(&mbdb->irq_spinlock);

	mbdb_init(mbdb);

	if (mbdb->polling) {
		timer_setup(&mbdb->inbox_timer, inbox_timer_fn, 0);
		add_timer(&mbdb->inbox_timer);
	}

	return 0;
}
