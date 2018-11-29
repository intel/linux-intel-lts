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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *
 */
#ifndef __KMB_REGS_H__
#define __KMB_REGS_H__

/*LCD CONTROLLER REGISTERS */
#define LCD_CONTROL			(0x4 * 0x000)
#define LCD_INT_STATUS			(0x4 * 0x001)
#define LCD_INT_ENABLE			(0x4 * 0x002)
#define LCD_INT_CLEAR			(0x4 * 0x003)
#define LCD_LINE_COUNT			(0x4 * 0x004)
#define LCD_LINE_COMPARE		(0x4 * 0x005)
#define LCD_VSTATUS			(0x4 * 0x006)
#define LCD_VSTATUS_COMPARE		(0x4 * 0x007)
#define LCD_SCREEN_WIDTH		(0x4 * 0x008)
#define LCD_SCREEN_HEIGHT		(0x4 * 0x009)
#define LCD_FIELD_INT_CFG		(0x4 * 0x00a)
#define LCD_FIFO_FLUSH			(0x4 * 0x00b)
#define LCD_BG_COLOUR_LS		(0x4 * 0x00c)
#define LCD_BG_COLOUR_MS		(0x4 * 0x00d)
#define LCD_RAM_CFG		        (0x4 * 0x00e)
#define LCD_LAYER0_CFG		        (0x4 * 0x100)
#define LCD_LAYERn_CFG(N)		(LCD_LAYER0_CFG + (0x400*N))
#define LCD_LAYER0_COL_START		(0x4 * 0x101)
#define LCD_LAYERn_COL_START(N)		(LCD_LAYER0_COL_START + (0x400*N))
#define LCD_LAYER0_ROW_START		(0x4 * 0x102)
#define LCD_LAYERn_ROW_START(N)		(LCD_LAYER0_ROW_START + (0x400*N))
#define LCD_LAYER0_WIDTH	        (0x4 * 0x103)
#define LCD_LAYERn_WIDTH(N)		(LCD_LAYER0_WIDTH + (0x400*N))
#define LCD_LAYER0_HEIGHT		(0x4 * 0x104)
#define LCD_LAYERn_HEIGHT(N)		(LCD_LAYER0_HEIGHT + (0x400*N))
#define LCD_LAYER0_SCALE_CFG		(0x4 * 0x105)
#define LCD_LAYERn_SCALE_CFG(N)		(LCD_LAYER0_SCALE_CFG + (0x400*N))
#define LCD_LAYER0_ALPHA	        (0x4 * 0x106)
#define LCD_LAYERn_ALPHA(N)		(LCD_LAYER0_ALPHA + (0x400*N))
#define LCD_LAYER0_INV_COLOUR_LS	(0x4 * 0x107)
#define LCD_LAYERn_INV_COLOUR_LS(N)	(LCD_LAYER0_INV_COLOUR_LS + (0x400*N))
#define LCD_LAYER0_INV_COLOUR_MS	(0x4 * 0x108)
#define LCD_LAYERn_INV_COLOUR_MS(N)	(LCD_LAYER0_INV_COLOUR_MS + (0x400*N))
#define LCD_LAYER0_TRANS_COLOUR_LS	(0x4 * 0x109)
#define LCD_LAYERn_TRANS_COLOUR_LS(N)	(LCD_LAYER0_TRANS_COLOUR_LS + (0x400*N))
#define LCD_LAYER0_TRANS_COLOUR_MS	(0x4 * 0x10a)
#define LCD_LAYERn_TRANS_COLOUR_MS(N)	(LCD_LAYER0_TRANS_COLOUR_MS + (0x400*N))
#define LCD_LAYER0_CSC_COEFF11		(0x4 * 0x10b)
#define LCD_LAYERn_CSC_COEFF11(N)	(LCD_LAYER0_CSC_COEFF11 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF12		(0x4 * 0x10c)
#define LCD_LAYERn_CSC_COEFF12(N)	(LCD_LAYER0_CSC_COEFF12 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF13		(0x4 * 0x10d)
#define LCD_LAYERn_CSC_COEFF13(N)	(LCD_LAYER0_CSC_COEFF13 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF21		(0x4 * 0x10e)
#define LCD_LAYERn_CSC_COEFF21(N)	(LCD_LAYER0_CSC_COEFF21 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF22		(0x4 * 0x10f)
#define LCD_LAYERn_CSC_COEFF22(N)	(LCD_LAYER0_CSC_COEFF22 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF23		(0x4 * 0x110)
#define LCD_LAYERn_CSC_COEFF23(N)	(LCD_LAYER0_CSC_COEFF23 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF31		(0x4 * 0x111)
#define LCD_LAYERn_CSC_COEFF31(N)	(LCD_LAYER0_CSC_COEFF31 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF32		(0x4 * 0x112)
#define LCD_LAYERn_CSC_COEFF32(N)	  (LCD_LAYER0_CSC_COEFF32 + (0x400*N))
#define LCD_LAYER0_CSC_COEFF33		(0x4 * 0x113)
#define LCD_LAYERn_CSC_COEFF33(N)	(LCD_LAYER0_CSC_COEFF33 + (0x400*N))
#define LCD_LAYER0_CSC_OFF1		(0x4 * 0x114)
#define LCD_LAYERn_CSC_OFF1(N)		(LCD_LAYER0_CSC_OFF1 + (0x400*N))
#define LCD_LAYER0_CSC_OFF2		(0x4 * 0x115)
#define LCD_LAYERn_CSC_OFF2(N)		(LCD_LAYER0_CSC_OFF2 + (0x400*N))
#define LCD_LAYER0_CSC_OFF3		(0x4 * 0x116)
#define LCD_LAYERn_CSC_OFF3(N)		(LCD_LAYER0_CSC_OFF3 + (0x400*N))
#define LCD_LAYER0_DMA_CFG		(0x4 * 0x117)
#define LCD_LAYERn_DMA_CFG(N)		(LCD_LAYER0_DMA_CFG + (0x400*N))
#define LCD_LAYER0_DMA_START_ADR	(0x4 * 0x118)
#define LCD_LAYERn_DMA_START_ADDR(N)	(LCD_LAYER0_DMA_START_ADR + (0x400*N))
#define LCD_LAYER0_DMA_START_SHADOW	(0x4 * 0x119)
#define LCD_LAYERn_DMA_START_SHADOW(N)	(LCD_LAYER0_DMA_START_SHADOW + (0x400*N))
#define LCD_LAYER0_DMA_LEN		(0x4 * 0x11a)
#define LCD_LAYERn_DMA_LEN(N)		(LCD_LAYER0_DMA_LEN + (0x400*N))
#define LCD_LAYER0_DMA_LEN_SHADOW	(0x4 * 0x11b)
#define LCD_LAYERn_DMA_LEN_SHADOW(N)	(LCD_LAYER0_DMA_LEN_SHADOW + (0x400*N))
#define LCD_LAYER0_DMA_STATUS		(0x4 * 0x11c)
#define LCD_LAYERn_DMA_STATUS(N)	(LCD_LAYER0_DMA_STATUS + (0x400*N))
#define LCD_LAYER0_DMA_LINE_WIDTH	(0x4 * 0x11d)
#define LCD_LAYERn_DMA_LINE_WIDTH(N)	(LCD_LAYER0_DMA_LINE_WIDTH + (0x400*N))
#define LCD_LAYER0_DMA_LINE_VSTRIDE	(0x4 * 0x11e)
#define LCD_LAYERn_DMA_LINE_VSTRIDE(N)	(LCD_LAYER0_DMA_LINE_VSTRIDE + (0x400*N))
#define LCD_LAYER0_DMA_FIFO_STATUS	(0x4 * 0x11f)
#define LCD_LAYERn_DMA_FIFO_STATUS(N)	(LCD_LAYER0_DMA_FIFO_STATUS + (0x400*N))
#define LCD_LAYER0_CFG2			(0x4 * 0x120)
#define LCD_LAYERn_CFG2(N)		(LCD_LAYER0_CFG2 + (0x400*N))

#define LCD_LAYER1_CFG			(0x4 * 0x200)
#define LCD_LAYERn_CFG2(N)		(LCD_LAYER0_CFG2 + (0x400*N))
#define LCD_LAYER1_COL_START		(0x4 * 0x201)
#define LCD_LAYER1_ROW_START		(0x4 * 0x202)
#define LCD_LAYER1_WIDTH		(0x4 * 0x203)
#define LCD_LAYER1_HEIGHT		(0x4 * 0x204)
#define LCD_LAYER1_SCALE_CFG		(0x4 * 0x205)
#define LCD_LAYER1_ALPHA		(0x4 * 0x206)
#define LCD_LAYER1_INV_COLOUR_LS	(0x4 * 0x207)
#define LCD_LAYER1_INV_COLOUR_MS	(0x4 * 0x208)
#define LCD_LAYER1_TRANS_COLOUR_LS	(0x4 * 0x209)
#define LCD_LAYER1_TRANS_COLOUR_MS	(0x4 * 0x20a)
#define LCD_LAYER1_CSC_COEFF11		(0x4 * 0x20b)
#define LCD_LAYER1_CSC_COEFF12		(0x4 * 0x20c)
#define LCD_LAYER1_CSC_COEFF13		(0x4 * 0x20d)
#define LCD_LAYER1_CSC_COEFF21		(0x4 * 0x20e)
#define LCD_LAYER1_CSC_COEFF22		(0x4 * 0x20f)
#define LCD_LAYER1_CSC_COEFF23		(0x4 * 0x210)
#define LCD_LAYER1_CSC_COEFF31		(0x4 * 0x211)
#define LCD_LAYER1_CSC_COEFF32		(0x4 * 0x212)
#define LCD_LAYER1_CSC_COEFF33		(0x4 * 0x213)
#define LCD_LAYER1_CSC_OFF1		(0x4 * 0x214)
#define LCD_LAYER1_CSC_OFF2		(0x4 * 0x215)
#define LCD_LAYER1_CSC_OFF3		(0x4 * 0x216)
#define LCD_LAYER1_DMA_CFG		(0x4 * 0x217)
#define LCD_LAYER1_DMA_START_ADR	(0x4 * 0x218)
#define LCD_LAYER1_DMA_START_SHADOW	(0x4 * 0x219)
#define LCD_LAYER1_DMA_LEN		(0x4 * 0x21a)
#define LCD_LAYER1_DMA_LEN_SHADOW	(0x4 * 0x21b)
#define LCD_LAYER1_DMA_STATUS		(0x4 * 0x21c)
#define LCD_LAYER1_DMA_LINE_WIDTH	(0x4 * 0x21d)
#define LCD_LAYER1_DMA_LINE_VSTRIDE	(0x4 * 0x21e)
#define LCD_LAYER1_DMA_FIFO_STATUS	(0x4 * 0x21f)
#define LCD_LAYER1_CFG2			(0x4 * 0x220)
#define LCD_LAYER2_CFG			(0x4 * 0x300)
#define LCD_LAYER2_COL_START		(0x4 * 0x301)
#define LCD_LAYER2_ROW_START		(0x4 * 0x302)
#define LCD_LAYER2_WIDTH		(0x4 * 0x303)
#define LCD_LAYER2_HEIGHT		(0x4 * 0x304)
#define LCD_LAYER2_SCALE_CFG		(0x4 * 0x305)
#define LCD_LAYER2_ALPHA		(0x4 * 0x306)
#define LCD_LAYER2_INV_COLOUR_LS	(0x4 * 0x307)
#define LCD_LAYER2_INV_COLOUR_MS	(0x4 * 0x308)
#define LCD_LAYER2_TRANS_COLOUR_LS	(0x4 * 0x309)
#define LCD_LAYER2_TRANS_COLOUR_MS	(0x4 * 0x30a)
#define LCD_LAYER2_CSC_COEFF11		(0x4 * 0x30b)
#define LCD_LAYER2_CSC_COEFF12		(0x4 * 0x30c)
#define LCD_LAYER2_CSC_COEFF13		(0x4 * 0x30d)
#define LCD_LAYER2_CSC_COEFF21		(0x4 * 0x30e)
#define LCD_LAYER2_CSC_COEFF22		(0x4 * 0x30f)
#define LCD_LAYER2_CSC_COEFF23		(0x4 * 0x310)
#define LCD_LAYER2_CSC_COEFF31		(0x4 * 0x311)
#define LCD_LAYER2_CSC_COEFF32		(0x4 * 0x312)
#define LCD_LAYER2_CSC_COEFF33		(0x4 * 0x313)
#define LCD_LAYER2_CSC_OFF1		(0x4 * 0x314)
#define LCD_LAYER2_CSC_OFF2		(0x4 * 0x315)
#define LCD_LAYER2_CSC_OFF3		(0x4 * 0x316)
#define LCD_LAYER2_DMA_CFG		(0x4 * 0x317)
#define LCD_LAYER2_DMA_START_ADR	(0x4 * 0x318)
#define LCD_LAYER2_DMA_START_SHADOW	(0x4 * 0x319)
#define LCD_LAYER2_DMA_LEN		(0x4 * 0x31a)
#define LCD_LAYER2_DMA_LEN_SHADOW	(0x4 * 0x31b)
#define LCD_LAYER2_DMA_STATUS		(0x4 * 0x31c)
#define LCD_LAYER2_DMA_LINE_WIDTH	(0x4 * 0x31d)
#define LCD_LAYER2_DMA_LINE_VSTRIDE	(0x4 * 0x31e)
#define LCD_LAYER2_DMA_FIFO_STATUS	(0x4 * 0x31f)
#define LCD_LAYER2_CFG2			(0x4 * 0x320)
#define LCD_LAYER3_CFG			(0x4 * 0x400)
#define LCD_LAYER3_COL_START		(0x4 * 0x401)
#define LCD_LAYER3_ROW_START		(0x4 * 0x402)
#define LCD_LAYER3_WIDTH		(0x4 * 0x403)
#define LCD_LAYER3_HEIGHT		(0x4 * 0x404)
#define LCD_LAYER3_SCALE_CFG		(0x4 * 0x405)
#define LCD_LAYER3_ALPHA		(0x4 * 0x406)
#define LCD_LAYER3_INV_COLOUR_LS	(0x4 * 0x407)
#define LCD_LAYER3_INV_COLOUR_MS	(0x4 * 0x408)
#define LCD_LAYER3_TRANS_COLOUR_LS	(0x4 * 0x409)
#define LCD_LAYER3_TRANS_COLOUR_MS	(0x4 * 0x40a)
#define LCD_LAYER3_CSC_COEFF11		(0x4 * 0x40b)
#define LCD_LAYER3_CSC_COEFF12		(0x4 * 0x40c)
#define LCD_LAYER3_CSC_COEFF13		(0x4 * 0x40d)
#define LCD_LAYER3_CSC_COEFF21		(0x4 * 0x40e)
#define LCD_LAYER3_CSC_COEFF22		(0x4 * 0x40f)
#define LCD_LAYER3_CSC_COEFF23		(0x4 * 0x410)
#define LCD_LAYER3_CSC_COEFF31		(0x4 * 0x411)
#define LCD_LAYER3_CSC_COEFF32		(0x4 * 0x412)
#define LCD_LAYER3_CSC_COEFF33		(0x4 * 0x413)
#define LCD_LAYER3_CSC_OFF1		(0x4 * 0x414)
#define LCD_LAYER3_CSC_OFF2		(0x4 * 0x415)
#define LCD_LAYER3_CSC_OFF3		(0x4 * 0x416)
#define LCD_LAYER3_DMA_CFG		(0x4 * 0x417)
#define LCD_LAYER3_DMA_START_ADR	(0x4 * 0x418)
#define LCD_LAYER3_DMA_START_SHADOW	(0x4 * 0x419)
#define LCD_LAYER3_DMA_LEN		(0x4 * 0x41a)
#define LCD_LAYER3_DMA_LEN_SHADOW	(0x4 * 0x41b)
#define LCD_LAYER3_DMA_STATUS		(0x4 * 0x41c)
#define LCD_LAYER3_DMA_LINE_WIDTH	(0x4 * 0x41d)
#define LCD_LAYER3_DMA_LINE_VSTRIDE	(0x4 * 0x41e)
#define LCD_LAYER3_DMA_FIFO_STATUS	(0x4 * 0x41f)
#define LCD_LAYER3_CFG2			(0x4 * 0x420)
#define LCD_LAYER2_CLUT0		(0x4 * 0x500)
#define LCD_LAYER2_CLUT1		(0x4 * 0x501)
#define LCD_LAYER2_CLUT2		(0x4 * 0x502)
#define LCD_LAYER2_CLUT3		(0x4 * 0x503)
#define LCD_LAYER2_CLUT4		(0x4 * 0x504)
#define LCD_LAYER2_CLUT5		(0x4 * 0x505)
#define LCD_LAYER2_CLUT6		(0x4 * 0x506)
#define LCD_LAYER2_CLUT7		(0x4 * 0x507)
#define LCD_LAYER2_CLUT8		(0x4 * 0x508)
#define LCD_LAYER2_CLUT9		(0x4 * 0x509)
#define LCD_LAYER2_CLUT10		(0x4 * 0x50a)
#define LCD_LAYER2_CLUT11		(0x4 * 0x50b)
#define LCD_LAYER2_CLUT12		(0x4 * 0x50c)
#define LCD_LAYER2_CLUT13		(0x4 * 0x50d)
#define LCD_LAYER2_CLUT14		(0x4 * 0x50e)
#define LCD_LAYER2_CLUT15		(0x4 * 0x50f)
#define LCD_LAYER3_CLUT0		(0x4 * 0x600)
#define LCD_LAYER3_CLUT1		(0x4 * 0x601)
#define LCD_LAYER3_CLUT2		(0x4 * 0x602)
#define LCD_LAYER3_CLUT3		(0x4 * 0x603)
#define LCD_LAYER3_CLUT4		(0x4 * 0x604)
#define LCD_LAYER3_CLUT5		(0x4 * 0x605)
#define LCD_LAYER3_CLUT6		(0x4 * 0x606)
#define LCD_LAYER3_CLUT7		(0x4 * 0x607)
#define LCD_LAYER3_CLUT8		(0x4 * 0x608)
#define LCD_LAYER3_CLUT9		(0x4 * 0x609)
#define LCD_LAYER3_CLUT10		(0x4 * 0x60a)
#define LCD_LAYER3_CLUT11		(0x4 * 0x60b)
#define LCD_LAYER3_CLUT12		(0x4 * 0x60c)
#define LCD_LAYER3_CLUT13		(0x4 * 0x60d)
#define LCD_LAYER3_CLUT14		(0x4 * 0x60e)
#define LCD_LAYER3_CLUT15		(0x4 * 0x60f)
#define LCD_LAYER0_DMA_START_CB_ADR	(0x4 * 0x700)
#define LCD_LAYER0_DMA_START_CB_SHADOW	(0x4 * 0x701)
#define LCD_LAYER0_DMA_CB_LINE_WIDTH	(0x4 * 0x702)
#define LCD_LAYER0_DMA_CB_LINE_VSTRIDE	(0x4 * 0x703)
#define LCD_LAYER0_DMA_START_CR_ADR	(0x4 * 0x704)
#define LCD_LAYER0_DMA_START_CR_SHADOW	(0x4 * 0x705)
#define LCD_LAYER0_DMA_CR_LINE_WIDTH	(0x4 * 0x706)
#define LCD_LAYER0_DMA_CR_LINE_VSTRIDE	(0x4 * 0x707)
#define LCD_LAYER1_DMA_START_CB_ADR	(0x4 * 0x708)
#define LCD_LAYER1_DMA_START_CB_SHADOW	(0x4 * 0x709)
#define LCD_LAYER1_DMA_CB_LINE_WIDTH	(0x4 * 0x70a)
#define LCD_LAYER1_DMA_CB_LINE_VSTRIDE	(0x4 * 0x70b)
#define LCD_LAYER1_DMA_START_CR_ADR	(0x4 * 0x70c)
#define LCD_LAYER1_DMA_START_CR_SHADOW	(0x4 * 0x70d)
#define LCD_LAYER1_DMA_CR_LINE_WIDTH	(0x4 * 0x70e)
#define LCD_LAYER1_DMA_CR_LINE_VSTRIDE	(0x4 * 0x70f)
#define LCD_OUT_FORMAT_CFG		(0x4 * 0x800)
#define LCD_HSYNC_WIDTH			(0x4 * 0x801)
#define LCD_H_BACKPORCH			(0x4 * 0x802)
#define LCD_H_ACTIVEWIDTH		(0x4 * 0x803)
#define LCD_H_FRONTPORCH		(0x4 * 0x804)
#define LCD_VSYNC_WIDTH			(0x4 * 0x805)
#define LCD_V_BACKPORCH			(0x4 * 0x806)
#define LCD_V_ACTIVEHEIGHT		(0x4 * 0x807)
#define LCD_V_FRONTPORCH		(0x4 * 0x808)
#define LCD_VSYNC_START			(0x4 * 0x809)
#define LCD_VSYNC_END			(0x4 * 0x80a)
#define LCD_V_BACKPORCH_EVEN		(0x4 * 0x80b)
#define LCD_VSYNC_WIDTH_EVEN		(0x4 * 0x80c)
#define LCD_V_ACTIVEHEIGHT_EVEN		(0x4 * 0x80d)
#define LCD_V_FRONTPORCH_EVEN		(0x4 * 0x80e)
#define LCD_VSYNC_START_EVEN		(0x4 * 0x80f)
#define LCD_VSYNC_END_EVEN		(0x4 * 0x810)
#define LCD_TIMING_GEN_TRIG		(0x4 * 0x811)
#define LCD_PWM0_CTRL			(0x4 * 0x812)
#define LCD_PWM0_RPT_LEADIN		(0x4 * 0x813)
#define LCD_PWM0_HIGH_LOW		(0x4 * 0x814)
#define LCD_PWM1_CTRL			(0x4 * 0x815)
#define LCD_PWM1_RPT_LEADIN		(0x4 * 0x816)
#define LCD_PWM1_HIGH_LOW		(0x4 * 0x817)
#define LCD_PWM2_CTRL			(0x4 * 0x818)
#define LCD_PWM2_RPT_LEADIN		(0x4 * 0x819)
#define LCD_PWM2_HIGH_LOW		(0x4 * 0x81a)
#define LCD_VIDEO0_DMA0_BYTES		(0x4 * 0xb00)
#define LCD_VIDEO0_DMA0_STATE		(0x4 * 0xb01)
#define LCD_VIDEO0_DMA1_BYTES		(0x4 * 0xb02)
#define LCD_VIDEO0_DMA1_STATE		(0x4 * 0xb03)
#define LCD_VIDEO0_DMA2_BYTES		(0x4 * 0xb04)
#define LCD_VIDEO0_DMA2_STATE		(0x4 * 0xb05)
#define LCD_VIDEO1_DMA0_BYTES		(0x4 * 0xb06)
#define LCD_VIDEO1_DMA0_STATE		(0x4 * 0xb07)
#define LCD_VIDEO1_DMA1_BYTES		(0x4 * 0xb08)
#define LCD_VIDEO1_DMA1_STATE		(0x4 * 0xb09)
#define LCD_VIDEO1_DMA2_BYTES		(0x4 * 0xb0a)
#define LCD_VIDEO1_DMA2_STATE		(0x4 * 0xb0b)
#define LCD_GRAPHIC0_DMA_BYTES		(0x4 * 0xb0c)
#define LCD_GRAPHIC0_DMA_STATE		(0x4 * 0xb0d)
#define LCD_GRAPHIC1_DMA_BYTES		(0x4 * 0xb0e)
#define LCD_GRAPHIC1_DMA_STATE		(0x4 * 0xb0f)

#define LAYER3_DMA_FIFO_UNDERFLOW_BIT		(1<<26)
#define LAYER3_DMA_OVERFLOW_BIT			(1<<25)
#define LAYER3_DMA_IDLE_BIT			(1<<24)
#define LAYER3_DMA_DONE_BIT			(1<<23)

#define LAYER2_DMA_FIFO_UNDERFLOW_BIT		(1<<22)
#define LAYER2_DMA_OVERFLOW_BIT			(1<<21)
#define LAYER2_DMA_IDLE_BIT			(1<<20)
#define LAYER2_DMA_DONE_BIT			(1<<19)

#define LAYER1_DMA_CR_FIFO_UNDERFLOW_BIT	(1<<18)
#define LAYER1_DMA_CR_FIFO_OVERFLOW_BIT		(1<<17)
#define LAYER1_DMA_CB_FIFO_UNDERFLOW_BIT	(1<<16)
#define LAYER1_DMA_CB_FIFO_OVERFLOW_BIT		(1<<15)

#define LAYER1_DMA_FIFO_UNDERFLOW_BIT		(1<<14)
#define LAYER1_DMA_OVERFLOW_BIT			(1<<13)
#define LAYER1_DMA_IDLE_BIT			(1<<12)
#define LAYER1_DMA_DONE_BIT			(1<<11)

#define LAYER0_DMA_CR_FIFO_UNDERFLOW_BIT	(1<<10)
#define LAYER0_DMA_CR_FIFO_OVERFLOW_BIT		(1<<9)
#define LAYER0_DMA_CB_FIFO_UNDERFLOW_BIT	(1<<8)
#define LAYER0_DMA_CB_FIFO_OVERFLOW_BIT		(1<<7)

#define LAYER0_DMA_FIFO_UNDEFLOW_BIT		(1<<6)
#define LAYER0_DMA_OVERFLOW_BIT			(1<<5)
#define LAYER0_DMA_IDLE_BIT			(1<<4)
#define LAYER0_DMA_DONE_BIT			(1<<3)

#define  BLT_VIDEOn_DMAm_STATE			0x00
#define  BLT_VIDEOn_DMAm_BYTES			0x00
#define  BLT_RAM_CFG				0x00

#define  BLT_LAYERn_WIDTH(N)			(0x40C + (0x400*N))
#define  BLT_LAYERn_HEIGHT_OFFSET(N)		(0x410 + (0x400*N))

#define  BLT_LAYERn_TRANS_COLOUR_MS		0x0
#define  BLT_LAYERn_TRANS_COLOUR_LS		0x0
#define  BLT_LAYERn_SCALE_CFG			0x0
#define  BLT_LAYERn_ROW_START			0x0
#define  BLT_LAYERn_INV_COLOUR_MS		0x0
#define  BLT_LAYERn_INV_COLOUR_LS		0x0

/*  LCD controller Layer DMA config register */

/* bit 0 default is disabled */
#define LCD_DMA_LAYER_ENABLE			(0x001)
/* bit 1 this should be used only as a mask when reading the status from
 * the DMA CFG register
 */
#define LCD_DMA_LAYER_STATUS			(0x002)
/* bit 2 */
#define LCD_DMA_LAYER_AUTO_UPDATE		(0x004)
/* bit 3 */
#define LCD_DMA_LAYER_CONT_UPDATE		(0x008)
/* bit 2 + bit 3 */
#define LCD_DMA_LAYER_CONT_PING_PONG_UPDATE	(0x00C)
/* bit 4 set FIFO addressing mode, default is increment after each burst */
#define LCD_DMA_LAYER_FIFO_ADR_MODE		(0x010)
/* bit 5:9 default axi burst is 1 */
#define LCD_DMA_LAYER_AXI_BURST_1		(0x020)
#define LCD_DMA_LAYER_AXI_BURST_2		(0x040)
#define LCD_DMA_LAYER_AXI_BURST_3		(0x060)
#define LCD_DMA_LAYER_AXI_BURST_4		(0x080)
#define LCD_DMA_LAYER_AXI_BURST_5		(0x0A0)
#define LCD_DMA_LAYER_AXI_BURST_6		(0x0C0)
#define LCD_DMA_LAYER_AXI_BURST_7		(0x0E0)
#define LCD_DMA_LAYER_AXI_BURST_8		(0x100)
#define LCD_DMA_LAYER_AXI_BURST_9		(0x120)
#define LCD_DMA_LAYER_AXI_BURST_10		(0x140)
#define LCD_DMA_LAYER_AXI_BURST_11		(0x160)
#define LCD_DMA_LAYER_AXI_BURST_12		(0x180)
#define LCD_DMA_LAYER_AXI_BURST_13		(0x1A0)
#define LCD_DMA_LAYER_AXI_BURST_14		(0x1C0)
#define LCD_DMA_LAYER_AXI_BURST_15		(0x1E0)
#define LCD_DMA_LAYER_AXI_BURST_16		(0x200)
/* bit 10 */
#define LCD_DMA_LAYER_V_STRIDE_EN		(0x400)

/* **************************************************************************
 *			LCD controller control register defines
 ****************************************************************************
 */
/* --- bit 0 */
#define LCD_CTRL_PROGRESSIVE		(0x00)	/* default */
#define LCD_CTRL_INTERLACED		(0x01)
/* --- bit 1 */
#define LCD_CTRL_ENABLE			(0x02)	/* enable conrtoller */
/* --- bits 2,3,4,5 */
#define LCD_CTRL_VL1_ENABLE		(0x04)	/* enable video layer 1 */
#define LCD_CTRL_VL2_ENABLE		(0x08)	/* enable  video layer 2 */
#define LCD_CTRL_GL1_ENABLE		(0x10)	/* enable  graphics layer 1 */
#define LCD_CTRL_GL2_ENABLE		(0x20)	/* enable  graphics layer 2 */
/* --- bits 6:7 */
#define LCD_CTRL_ALPHA_BLEND_VL1	(0x00)	/* video layer 1 - default */
#define LCD_CTRL_ALPHA_BLEND_VL2	(0x40)	/* video layer 2 */
#define LCD_CTRL_ALPHA_BLEND_GL1	(0x80)	/* graphics layer 1 */
#define LCD_CTRL_ALPHA_BLEND_GL2	(0xC0)	/* graphics layer 2 */
/* --- bits 8:9 */
#define LCD_CTRL_ALPHA_TOP_VL1		(0x000)	/* video layer 1 - default */
#define LCD_CTRL_ALPHA_TOP_VL2		(0x100)	/* video layer 2 */
#define LCD_CTRL_ALPHA_TOP_GL1		(0x200)	/* graphics layer 1 */
#define LCD_CTRL_ALPHA_TOP_GL2		(0x300)	/* graphics layer 2 */
/* --- bits 10:11 */
#define LCD_CTRL_ALPHA_MIDDLE_VL1	(0x000)	/* video layer 1 - default */
#define LCD_CTRL_ALPHA_MIDDLE_VL2	(0x400)	/* video layer 2 */
#define LCD_CTRL_ALPHA_MIDDLE_GL1	(0x800)	/* graphics layer 1 */
#define LCD_CTRL_ALPHA_MIDDLE_GL2	(0xC00)	/* graphics layer 2 */
/* --- bits 12:13 */
#define LCD_CTRL_ALPHA_BOTTOM_VL1	(0x0000)	/* video layer 1 */
#define LCD_CTRL_ALPHA_BOTTOM_VL2	(0x1000)	/* video layer 2 */
#define LCD_CTRL_ALPHA_BOTTOM_GL1	(0x2000)	/* graphics layer 1 */
#define LCD_CTRL_ALPHA_BOTTOM_GL2	(0x3000)	/* graphics layer 2 */
/* --- bit 14 */
#define LCD_CTRL_TIM_GEN_ENABLE		(0x4000)	/* timing generator */
/* --- bit 15 */
#define LCD_CTRL_DISPLAY_MODE_ONE_SHOT	(0x8000)	/* default continuous */
/* --- bits 16, 17, 18 */
#define LCD_CTRL_PWM0_EN		(0x10000)	/* enable PWM 0 */
#define LCD_CTRL_PWM1_EN		(0x20000)	/* enable PWM 1 */
#define LCD_CTRL_PWM2_EN		(0x40000)	/* enable PWM 2 */
/* --- bits 19:20 */
#define LCD_CTRL_OUTPUT_DISABLED	(0x000000)	/* output disabled */
#define LCD_CTRL_OUTPUT_ENABLED		(0x080000)
/* --- bit 21 */
#define LCD_CTRL_SHARP_TFT		(0x200000)
/* = bit 21 VSYNC BACK PORCH LEVEL */
#define LCD_CTRL_BPORCH_ENABLE		(0x00200000)
/* = bit 22 VSYNC FRONT PORCH LEVEL */
#define LCD_CTRL_FPORCH_ENABLE		(0x00400000)
/* = bit 28 enable pipelined (outstanding) DMA reads */
#define LCD_CTRL_PIPELINE_DMA		(0x10000000)

/* LCD Control register bit fields */

#define EIGHT_BITS			 8
#define SIXTEEN_BITS			 8
#define TWENTY_FOUR_BITS		 8
#define THIRT_TWO_BITS			 8

#define ENABLE				 1
/*LCD_VSTATUS_COMPARE Vertcal interval in which to generate vertcal
 * interval interrupt
 */
#define LCD_VSTATUS_VERTICAL_STATUS_MASK	 0x60	/* BITS 13 and 14 */
#define LCD_VSTATUS_COMPARE_VSYNC		 0x00
#define LCD_VSTATUS_COMPARE_BACKPORCH		 0x01
#define LCD_VSTATUS_COMPARE_ACTIVE		 0x10
#define LCD_VSTATUS_COMPARE_FRONT_PORCH		 0x11

/*interrupt bits */
#define LCD_INT_VERT_COMP			 (1 << 2)
#define LCD_INT_LINE_CMP			 (1 << 1)
#define LCD_INT_EOF				 (1 << 0)

#endif /* __KMB_REGS_H__ */
