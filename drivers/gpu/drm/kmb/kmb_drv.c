// SPDX-License-Identifier: GPL-2.0
/*
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
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/console.h>
#include <linux/list.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <drm/drm.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include "kmb_drv.h"
#include "kmb_crtc.h"
#include "kmb_plane.h"
#include "kmb_dsi.h"

#define DEBUG

/*IRQ handler*/
static irqreturn_t kmb_isr(int irq, void *arg);

static struct clk *clk_lcd;
static struct clk *clk_mipi;
static struct clk *clk_msscam;
static struct clk *clk_mipi_ecfg;
static struct clk *clk_mipi_cfg;

struct drm_bridge *adv_bridge;

static int kmb_display_clk_enable(void)
{
	int ret = 0;

	ret = clk_prepare_enable(clk_lcd);
	if (ret) {
		DRM_ERROR("Failed to enable LCD clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(clk_mipi);
	if (ret) {
		DRM_ERROR("Failed to enable MIPI clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(clk_msscam);
	if (ret) {
		DRM_ERROR("Failed to enable MSSCAM clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(clk_mipi_ecfg);
	if (ret) {
		DRM_ERROR("Failed to enable MIPI_ECFG clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(clk_mipi_cfg);
	if (ret) {
		DRM_ERROR("Failed to enable MIPI_CFG clock: %d\n", ret);
		return ret;
	}
	DRM_INFO("SUCCESS : enabled LCD MIPI clocks\n");
	return 0;
}

static int kmb_display_clk_disable(void)
{
	if (clk_lcd)
		clk_disable_unprepare(clk_lcd);
	if (clk_mipi)
		clk_disable_unprepare(clk_mipi);
	if (clk_msscam)
		clk_disable_unprepare(clk_msscam);
	if (clk_mipi_ecfg)
		clk_disable_unprepare(clk_mipi_ecfg);
	if (clk_mipi_cfg)
		clk_disable_unprepare(clk_mipi_cfg);
	return 0;
}

static void __iomem *kmb_map_mmio(struct platform_device *pdev, char *name)
{
	struct resource *res;
	u32 size;
	void __iomem *mem;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		DRM_ERROR("failed to get resource for %s\n", name);
		return ERR_PTR(-ENOMEM);
	}
	size = resource_size(res);
	if (!request_mem_region(res->start, size, name)) {
		DRM_ERROR("failed to reserve %s registers\n", name);
		return ERR_PTR(-ENOMEM);
	}
	mem = ioremap_nocache(res->start, size);
	if (!mem) {
		DRM_ERROR("failed to ioremap %s registers\n", name);
		release_mem_region(res->start, size);
		return ERR_PTR(-ENOMEM);
	}
	return mem;
}

static int kmb_load(struct drm_device *drm, unsigned long flags)
{
	struct kmb_drm_private *dev_p = drm->dev_private;
	struct platform_device *pdev = to_platform_device(drm->dev);
#ifdef WIP
	/*u32 version;*/
	int irq_lcd, irq_mipi;
#endif
	int ret = 0;
	unsigned long clk;

	/* Map LCD MMIO registers */
	dev_p->lcd_mmio = kmb_map_mmio(pdev, "lcd_regs");
	if (IS_ERR(dev_p->lcd_mmio)) {
		DRM_ERROR("failed to map LCD registers\n");
		return -ENOMEM;
	}

	/* Map MIPI MMIO registers */
	dev_p->mipi_mmio = kmb_map_mmio(pdev, "mipi_regs");
	if (IS_ERR(dev_p->mipi_mmio)) {
		DRM_ERROR("failed to map MIPI registers\n");
		iounmap(dev_p->lcd_mmio);
		return -ENOMEM;
	}

	/* This is only for MIPI_TX_MSS_LCD_MIPI_CFG and MSS_CAM_CLK_CTRL
	 * register
	 */
	dev_p->msscam_mmio = kmb_map_mmio(pdev, "msscam_regs");
	if (IS_ERR(dev_p->msscam_mmio)) {
		DRM_ERROR("failed to map MSSCAM registers\n");
		iounmap(dev_p->lcd_mmio);
		iounmap(dev_p->mipi_mmio);
		return -ENOMEM;
	}

	/* Enable display clocks*/
	clk_lcd = clk_get(&pdev->dev, "clk_lcd");
	if (IS_ERR(clk_lcd)) {
		DRM_ERROR("clk_get() failed clk_lcd\n");
		goto setup_fail;
	}

	clk_mipi = clk_get(&pdev->dev, "clk_mipi");
	if (IS_ERR(clk_mipi)) {
		DRM_ERROR("clk_get() failed clk_mipi\n");
		goto setup_fail;
	}

	clk_msscam = clk_get(&pdev->dev, "clk_msscam");
	if (IS_ERR(clk_msscam)) {
		DRM_ERROR("clk_get() failed clk_msscam\n");
		goto setup_fail;
	}

	clk_mipi_ecfg = clk_get(&pdev->dev, "clk_mipi_ecfg");
	if (IS_ERR(clk_mipi_ecfg)) {
		DRM_ERROR("clk_get() failed clk_mipi_ecfg\n");
		goto setup_fail;
	}

	clk_mipi_cfg = clk_get(&pdev->dev, "clk_mipi_cfg");
	if (IS_ERR(clk_mipi_cfg)) {
		DRM_ERROR("clk_get() failed clk_mipi_cfg\n");
		goto setup_fail;
	}

	ret = kmb_display_clk_enable();

	/* Set LCD clock to 200 Mhz*/
	DRM_INFO("Get clk_lcd before set = %ld\n", clk_get_rate(clk_lcd));
	ret = clk_set_rate(clk_lcd, KMB_LCD_DEFAULT_CLK);
	if (clk_get_rate(clk_lcd) != KMB_LCD_DEFAULT_CLK) {
		DRM_ERROR("failed to set to clk_lcd to %d\n",
				KMB_LCD_DEFAULT_CLK);
		goto setup_fail;
	}
	DRM_INFO("Setting LCD clock to %d Mhz ret = %d\n",
			KMB_LCD_DEFAULT_CLK/1000000, ret);
	DRM_INFO("Get clk_lcd after set = %ld\n", clk_get_rate(clk_lcd));

	/* Set MIPI clock to 24 Mhz*/
	DRM_INFO("Get clk_mipi before set = %ld\n", clk_get_rate(clk_mipi));
	ret = clk_set_rate(clk_mipi, KMB_MIPI_DEFAULT_CLK);
	if (clk_get_rate(clk_mipi) != KMB_MIPI_DEFAULT_CLK) {
		DRM_ERROR("failed to set to clk_mipi to %d\n",
				KMB_MIPI_DEFAULT_CLK);
		goto setup_fail;
	}
	DRM_INFO("Setting MIPI clock to %d Mhz ret = %d\n",
			KMB_MIPI_DEFAULT_CLK/1000000, ret);
	DRM_INFO("Get clk_mipi after set = %ld\n", clk_get_rate(clk_mipi));

	clk = clk_get_rate(clk_mipi_ecfg);
	if (clk != KMB_MIPI_DEFAULT_CLK) {
		/* Set MIPI_ECFG clock to 24 Mhz*/
		DRM_INFO("Get clk_mipi_ecfg before set = %ld\n", clk);
		ret = clk_set_rate(clk_mipi_ecfg, KMB_MIPI_DEFAULT_CLK);
		clk = clk_get_rate(clk_mipi_ecfg);
		if (clk != KMB_MIPI_DEFAULT_CLK) {
			DRM_ERROR("failed to set to clk_mipi_ecfg to %d\n",
					KMB_MIPI_DEFAULT_CLK);
			goto setup_fail;
		}
		DRM_INFO("Setting MIPI_ECFG clock tp %d Mhz ret = %d\n",
				KMB_MIPI_DEFAULT_CLK/1000000, ret);
		DRM_INFO("Get clk_mipi_ecfg after set = %ld\n", clk);
	}

	clk = clk_get_rate(clk_mipi_cfg);
	if (clk != KMB_MIPI_DEFAULT_CLK) {
		/* Set MIPI_CFG clock to 24 Mhz*/
		DRM_INFO("Get clk_mipi_cfg before set = %ld\n", clk);
		ret = clk_set_rate(clk_mipi_cfg, 24000000);
		clk = clk_get_rate(clk_mipi_cfg);
		if (clk != KMB_MIPI_DEFAULT_CLK) {
			DRM_ERROR("failed to set to clk_mipi_cfg to %d\n",
					KMB_MIPI_DEFAULT_CLK);
			goto setup_fail;
		}
		DRM_INFO("Setting MIPI_CFG clock tp 24Mhz ret = %d\n", ret);
		DRM_INFO("Get clk_mipi_cfg after set = %ld\n", clk);
	}

	/* enable MSS_CAM_CLK_CTRL for MIPI TX and LCD */
	kmb_set_bitmask_msscam(dev_p, MSS_CAM_CLK_CTRL, LCD | MIPI_COMMON |
			MIPI_TX0);
#ifdef WIP
	/* Register irqs here - section 17.3 in databook
	 * lists LCD at 79 and 82 for MIPI under MSS CPU -
	 * firmware has to redirect it to A53
	 */
	DRM_INFO("platform_get_irq_byname %pOF\n", drm->dev->of_node);

	/* Allocate LCD interrupt resources, enable interrupt line,
	 * and setup IRQ handling
	 */
	irq_lcd = platform_get_irq_byname(pdev, "irq_lcd");
	if (irq_lcd < 0) {
		DRM_ERROR("irq_lcd not found");
		return irq_lcd;
	}

	pr_info("irq_lcd platform_get_irq = %d\n", irq_lcd);

	ret = request_irq(irq_lcd, kmb_isr, IRQF_SHARED, "irq_lcd", dev_p);
	dev_p->irq_lcd = irq_lcd;

	/* Allocate MIPI interrupt resources, enable interrupt line,
	 * and setup IRQ handling
	 */
	irq_mipi = platform_get_irq_byname(pdev, "irq_mipi");
	if (irq_mipi < 0) {
		DRM_ERROR("irq_mipi not found");
		return irq_mipi;
	}

	pr_info("irq_mipi platform_get_irq = %d\n", irq_mipi);
	ret = request_irq(irq_mipi, kmb_isr, IRQF_SHARED, "irq_mipi", dev_p);
	dev_p->irq_mipi = irq_mipi;

	/* TBD read and check for correct product version here */
#endif
	/* Get the optional framebuffer memory resource */
	ret = of_reserved_mem_device_init(drm->dev);
	if (ret && ret != -ENODEV)
		return ret;

	spin_lock_init(&dev_p->irq_lock);

	ret = kmb_setup_crtc(drm);
	if (ret < 0) {
		DRM_ERROR("failed to create crtc\n");
		goto setup_fail;
	}

	/* Initialize MIPI DSI */
	ret = kmb_dsi_init(drm, adv_bridge);
	if (ret) {
		DRM_ERROR("failed to initialize DSI\n");
		goto setup_fail;
	}

	DRM_INFO("%s : %d\n", __func__, __LINE__);
#ifdef WIP
	ret = drm_irq_install(drm, platform_get_irq(pdev, 0));
	if (ret < 0) {
		DRM_ERROR("failed to install IRQ handler\n");
		goto irq_fail;
	}
#endif
	return 0;

#ifdef WIP
irq_fail:
	drm_crtc_cleanup(&dev_p->crtc);
#endif
setup_fail:
	DRM_INFO("%s : %d\n", __func__, __LINE__);
	of_reserved_mem_device_release(drm->dev);

	return ret;
}

static const struct drm_mode_config_funcs kmb_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void kmb_setup_mode_config(struct drm_device *drm)
{
	drm_mode_config_init(drm);
	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;
	drm->mode_config.max_width = KMB_MAX_WIDTH;
	drm->mode_config.max_height = KMB_MAX_HEIGHT;
	drm->mode_config.funcs = &kmb_mode_config_funcs;
}


static irqreturn_t handle_lcd_irq(struct drm_device *dev)
{
	unsigned long status, val;

	status = kmb_read_lcd(dev->dev_private, LCD_INT_STATUS);
	if (status & LCD_INT_EOF) {
		/*To DO - handle EOF interrupt? */
		kmb_write_lcd(dev->dev_private, LCD_INT_CLEAR, LCD_INT_EOF);
	}
	if (status & LCD_INT_LINE_CMP) {
		/* clear line compare interrupt */
		kmb_write_lcd(dev->dev_private, LCD_INT_CLEAR,
				LCD_INT_LINE_CMP);
	}
	if (status & LCD_INT_VERT_COMP) {
		/* read VSTATUS */
		val = kmb_read_lcd(dev->dev_private, LCD_VSTATUS);
		val = (val & LCD_VSTATUS_VERTICAL_STATUS_MASK);
		switch (val) {
		case LCD_VSTATUS_COMPARE_VSYNC:
		case LCD_VSTATUS_COMPARE_BACKPORCH:
		case LCD_VSTATUS_COMPARE_ACTIVE:
		case LCD_VSTATUS_COMPARE_FRONT_PORCH:
			/* clear vertical compare interrupt */
			kmb_write_lcd(dev->dev_private, LCD_INT_CLEAR,
					LCD_INT_VERT_COMP);
			drm_handle_vblank(dev, 0);
			break;
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t  handle_mipi_irq(struct drm_device *dev)
{
	mipi_tx_handle_irqs(dev->dev_private);
	return IRQ_HANDLED;
}

static irqreturn_t kmb_isr(int irq, void *arg)
{
	struct drm_device *dev = (struct drm_device *)arg;
	struct kmb_drm_private *dev_p = dev->dev_private;
	irqreturn_t ret = IRQ_NONE;

	if (irq == dev_p->irq_lcd)
		ret = handle_lcd_irq(dev);
	else if (irq == dev_p->irq_mipi)
		ret = handle_mipi_irq(dev);

	return ret;
}

static void kmb_irq_reset(struct drm_device *drm)
{
	kmb_write_lcd(drm->dev_private, LCD_INT_CLEAR, 0xFFFF);
	kmb_write_lcd(drm->dev_private, LCD_INT_ENABLE, 0);
}

DEFINE_DRM_GEM_CMA_FOPS(fops);

static struct drm_driver kmb_driver = {
	.driver_features = DRIVER_HAVE_IRQ | DRIVER_GEM |
			DRIVER_MODESET | DRIVER_ATOMIC,
	.irq_handler = kmb_isr,
	.irq_preinstall = kmb_irq_reset,
	.irq_uninstall = kmb_irq_reset,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_print_info = drm_gem_cma_print_info,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.dumb_create = drm_gem_cma_dumb_create,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,
	.fops = &fops,
	.name = "kmb_display",
	.desc = "KEEMBAY DISPLAY DRIVER ",
	.date = "20190122",
	.major = 1,
	.minor = 0,
};

static void kmb_drm_unload(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct kmb_drm_private *dev_p = drm->dev_private;

	dump_stack();
	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	of_node_put(dev_p->crtc.port);
	dev_p->crtc.port = NULL;
	pm_runtime_get_sync(drm->dev);
	drm_irq_uninstall(drm);
	pm_runtime_put_sync(drm->dev);
	pm_runtime_disable(drm->dev);

	if (dev_p->lcd_mmio) {
		iounmap(dev_p->lcd_mmio);
		release_mem_region(LCD_BASE_ADDR, LCD_MMIO_SIZE);
	}

	if (dev_p->mipi_mmio) {
		iounmap(dev_p->mipi_mmio);
		release_mem_region(MIPI_BASE_ADDR, MIPI_MMIO_SIZE);
	}

	if (dev_p->msscam_mmio)
		iounmap(dev_p->msscam_mmio);

	of_reserved_mem_device_release(drm->dev);
	drm_mode_config_cleanup(drm);

	/*release clks */
	kmb_display_clk_disable();
	clk_put(clk_lcd);
	clk_put(clk_mipi);

	drm_dev_put(drm);
	drm->dev_private = NULL;
	dev_set_drvdata(dev, NULL);

	/* Unregister DSI host */
	dsi_host_unregister();
}

static int kmb_probe(struct platform_device *pdev)
{
	struct device *dev = get_device(&pdev->dev);
	struct drm_device *drm = NULL;
	struct kmb_drm_private *lcd;
	int ret = 0;

	/* The bridge (ADV 7535) will return -EPROBE_DEFER until it
	 * has a mipi_dsi_host to register its device to. So, we
	 * first register the DSI host during probe time, and then return
	 * -EPROBE_DEFER until the bridge is loaded. Probe will be called again
	 *  and then the rest of the driver initialization can procees
	 *  afterwards and the bridge can be successfully attached.
	 */
	adv_bridge =  kmb_dsi_host_bridge_init(dev);
	if (adv_bridge == ERR_PTR(-EPROBE_DEFER))
		return -EPROBE_DEFER;
	else if (IS_ERR(adv_bridge)) {
		DRM_ERROR("probe failed to initialize DSI host bridge\n");
		return PTR_ERR(adv_bridge);
	}

	/* Create DRM device */
	drm = drm_dev_alloc(&kmb_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	lcd = devm_kzalloc(dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	drm->dev_private = lcd;
	kmb_setup_mode_config(drm);
		dev_set_drvdata(dev, drm);

	/* Load driver */
	ret = kmb_load(drm, 0);
	if (ret == -EPROBE_DEFER) {
		DRM_INFO("wait for external bridge driver DT\n");
		return -EPROBE_DEFER;
	} else if (ret)
		goto err_free;

	/* Set the CRTC's port so that the encoder component can find it */
	lcd->crtc.port = of_graph_get_port_by_id(dev->of_node, 0);

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret < 0) {
		DRM_ERROR("failed to initialize vblank\n");
		goto err_vblank;
	}

	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);

	/* Register graphics device with the kernel */
	ret = drm_dev_register(drm, 0);

	lcd->n_layers = KMB_MAX_PLANES;
	if (ret)
		goto err_register;

	drm_fbdev_generic_setup(drm, 32);

	return 0;

err_register:
	drm_kms_helper_poll_fini(drm);
err_vblank:
	pm_runtime_disable(drm->dev);
err_free:
	drm_mode_config_cleanup(drm);
	dev_set_drvdata(dev, NULL);
	drm_dev_put(drm);
	dsi_host_unregister();

	return ret;
}

static int kmb_remove(struct platform_device *pdev)
{
	kmb_drm_unload(&pdev->dev);
	return 0;
}

static const struct of_device_id  kmb_of_match[] = {
	{.compatible = "intel,kmb_display"},
	{},
};

MODULE_DEVICE_TABLE(of, kmb_of_match);

static int __maybe_unused kmb_pm_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct kmb_drm_private *lcd = drm ? drm->dev_private : NULL;

	if (!lcd)
		return 0;

	drm_kms_helper_poll_disable(drm);

	lcd->state = drm_atomic_helper_suspend(drm);
	if (IS_ERR(lcd->state)) {
		drm_kms_helper_poll_enable(drm);
		return PTR_ERR(lcd->state);
	}

	return 0;
}

static int __maybe_unused kmb_pm_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct kmb_drm_private *lcd = drm ? drm->dev_private : NULL;

	if (!lcd)
		return 0;

	drm_atomic_helper_resume(drm, lcd->state);
	drm_kms_helper_poll_enable(drm);

	return 0;
}

static SIMPLE_DEV_PM_OPS(kmb_pm_ops, kmb_pm_suspend, kmb_pm_resume);

static struct platform_driver kmb_platform_driver = {
	.probe		= kmb_probe,
	.remove		= kmb_remove,
	.driver	= {
		.name = "kmb_display",
		.pm = &kmb_pm_ops,
		.of_match_table	= kmb_of_match,
	},
};

module_platform_driver(kmb_platform_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Keembay Display driver");
MODULE_LICENSE("GPL v2");
