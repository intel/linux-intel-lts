/* SPDX-License-Identifier: MIT
 *
 * Copyright © 2018 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 */
#ifndef __KMB_DRV_H__
#define __KMB_DRV_H__

#include "kmb_regs.h"

//#define FCCTEST
#define LCD_TEST
#define KMB_MAX_WIDTH			1920 /*max width in pixels */
#define KMB_MAX_HEIGHT			1080 /*max height in pixels */
#define KMB_MIN_WIDTH                   1920 /*max width in pixels */
#define KMB_MIN_HEIGHT                  1080 /*max height in pixels */
#define KMB_LCD_DEFAULT_CLK		200000000
#define KMB_MIPI_DEFAULT_CLK		24000000
#define KMB_MIPI_DEFAULT_CFG_CLK	24000000
#define KMB_SYS_CLK_MHZ			500

#define crtc_to_kmb_priv(x)	container_of(x, struct kmb_drm_private, crtc)

#define ICAM_MMIO		0x3b100000
#define ICAM_LCD_OFFSET		0x1080
#define ICAM_MMIO_SIZE		0x2000
struct kmb_drm_private {
	struct drm_device		drm;
	void __iomem			*lcd_mmio;
	void __iomem			*mipi_mmio;
	void __iomem			*msscam_mmio;
	void __iomem                    *icamlcd_mmio;
	unsigned char			n_layers;
	struct clk			*clk;
	struct drm_crtc			crtc;
	struct kmb_plane		*plane;
	struct drm_atomic_state		*state;
	spinlock_t			irq_lock;
	int				irq_lcd;
	int				irq_mipi;
	int				sys_clk_mhz;
	dma_addr_t			fb_addr;
};

static inline struct kmb_drm_private *to_kmb(const struct drm_device *dev)
{
	return container_of(dev, struct kmb_drm_private, drm);
}

struct blt_layer_config {
	unsigned char layer_format;
};

static inline void kmb_write_lcd(struct kmb_drm_private *dev_p,
				 unsigned int reg, u32 value)
{
#ifdef LCD_TEST
	writel(value, (dev_p->lcd_mmio + reg));
#endif
}

static inline void kmb_write_mipi(struct kmb_drm_private *dev_p,
				  unsigned int reg, u32 value)
{
	writel(value, (dev_p->mipi_mmio + reg));
}

static inline void kmb_write_msscam(struct kmb_drm_private *dev_p,
				    unsigned int reg, u32 value)
{
	writel(value, (dev_p->msscam_mmio + reg));
}

static inline u32 kmb_read_msscam(struct kmb_drm_private *dev_p,
				  unsigned int reg)
{
	return readl(dev_p->msscam_mmio + reg);
}

static inline void kmb_set_bitmask_msscam(struct kmb_drm_private *dev_p,
					  unsigned int reg, u32 mask)
{
	u32 reg_val = kmb_read_msscam(dev_p, reg);

	kmb_write_msscam(dev_p, reg, (reg_val | mask));
}

static inline u32 kmb_read_lcd(struct kmb_drm_private *dev_p, unsigned int reg)
{
#ifdef LCD_TEST
	return readl(dev_p->lcd_mmio + reg);
#endif
	return 0;
}

static inline void kmb_set_bitmask_lcd(struct kmb_drm_private *dev_p,
				       unsigned int reg, u32 mask)
{
#ifdef LCD_TEST
	u32 reg_val = kmb_read_lcd(dev_p, reg);

	kmb_write_lcd(dev_p, reg, (reg_val | mask));
#endif
}

static inline void kmb_clr_bitmask_lcd(struct kmb_drm_private *dev_p,
				       unsigned int reg, u32 mask)
{
#ifdef LCD_TEST
	u32 reg_val = kmb_read_lcd(dev_p, reg);

	kmb_write_lcd(dev_p, reg, (reg_val & (~mask)));
#endif
}

static inline u32 kmb_read_mipi(struct kmb_drm_private *dev_p, unsigned int reg)
{
	return readl(dev_p->mipi_mmio + reg);
}

static inline void kmb_write_bits_mipi(struct kmb_drm_private *dev_p,
				       unsigned int reg, u32 offset,
				       u32 num_bits, u32 value)
{
	u32 reg_val = kmb_read_mipi(dev_p, reg);
	u32 mask = (1 << num_bits) - 1;

	value &= mask;
	mask <<= offset;
	reg_val &= (~mask);
	reg_val |= (value << offset);
#ifdef DEBUG
	if (reg == DPHY_FREQ_CTRL0_3 + 4) {
		DRM_INFO("%s : %d  reg=0x%x offset=%d num_bits=%d val=0x%x\n",
			 __func__, __LINE__, reg, offset, num_bits,
			 reg_val);
	}
#endif
	kmb_write_mipi(dev_p, reg, reg_val);
}

static inline void kmb_set_bit_mipi(struct kmb_drm_private *dev_p,
				    unsigned int reg, u32 offset)
{
	u32 reg_val = kmb_read_mipi(dev_p, reg);

	kmb_write_mipi(dev_p, reg, reg_val | (1 << offset));
}

static inline void kmb_clr_bit_mipi(struct kmb_drm_private *dev_p,
				    unsigned int reg, u32 offset)
{
	u32 reg_val = kmb_read_mipi(dev_p, reg);

	kmb_write_mipi(dev_p, reg, reg_val & (~(1 << offset)));
}

static inline void kmb_set_bitmask_mipi(struct kmb_drm_private *dev_p,
					unsigned int reg, u32 mask)
{
	u32 reg_val = kmb_read_mipi(dev_p, reg);

	kmb_write_mipi(dev_p, reg, (reg_val | mask));
}

static inline void kmb_clr_bitmask_mipi(struct kmb_drm_private *dev_p,
					unsigned int reg, u32 mask)
{
	u32 reg_val = kmb_read_mipi(dev_p, reg);

	kmb_write_mipi(dev_p, reg, (reg_val & (~mask)));
}

int kmb_setup_crtc(struct drm_device *dev);
void kmb_set_scanout(struct kmb_drm_private *lcd);
#endif /* __KMB_DRV_H__ */
