#include <linux/console.h>
#include <asm/fixmap.h>
#include <asm/early_intel_th.h>
#include "sth.h"

static unsigned long sth_phys_addr;

void early_intel_th_init(const char *s)
{
	size_t n;
	unsigned long addr, chan;
	char buf[32] = {0, };
	char *match, *next;

	/* Expect ,0x<sw_bar>:<channel>[,keep] */
	if (*s == ',')
		++s;
	if (strncmp(s, "0x", 2))
		goto fail;

	n = strcspn(s, ",");
	if (n > sizeof(buf) - 1)
		goto fail;
	strncpy(buf, s, n);
	next = buf;

	/* Get sw_bar */
	match = strsep(&next, ":");
	if (!match)
		goto fail;

	if (kstrtoul(match, 16, &addr))
		goto fail;

	/* Get channel */
	if (kstrtoul(next, 0, &chan))
		goto fail;

	sth_phys_addr = addr + chan * sizeof(struct intel_th_channel);
	return;

fail:
	pr_err("%s invalid parameter %s", __func__, s);
}

static void intel_th_early_write(struct console *con, const char *buf,
				 unsigned len)
{
	struct intel_th_channel *channel;
	const u8 *p = buf;
	const u32 sven_header = 0x01000242;

	if (WARN_ON_ONCE(!sth_phys_addr))
		return;

	/* Software can send messages to Intel TH by writing to an MMIO space
	 * that is divided in several Master/Channel regions.
	 * Write directly to the address provided through the cmdline.
	 */
	set_fixmap_nocache(FIX_EARLYCON_MEM_BASE, sth_phys_addr);
	channel = (struct intel_th_channel *)
		(__fix_to_virt(FIX_EARLYCON_MEM_BASE) +
		 (sth_phys_addr & (PAGE_SIZE - 1)));

	/* Add hardcoded SVEN header
	 *  type: DEBUG_STRING
	 *  severity: SVEN_SEVERITY_NORMAL
	 *  length: payload size
	 *  subtype: SVEN_DEBUGSTR_Generic
	 */
	iowrite32(sven_header, &channel->DnTS);
	iowrite16(len, &channel->Dn);

	while (len) {
		if (len >= 4) {
			iowrite32(*(u32 *)p, &channel->Dn);
			p += 4;
			len -= 4;
		} else if (len >= 2) {
			iowrite16(*(u16 *)p, &channel->Dn);
			p += 2;
			len -= 2;
		} else {
			iowrite8(*(u8 *)p, &channel->Dn);
			p += 1;
			len -= 1;
		}
	}

	iowrite32(0, &channel->FLAG);
}

struct console intel_th_early_console = {
	.name = "earlyintelth",
	.write = intel_th_early_write,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};
