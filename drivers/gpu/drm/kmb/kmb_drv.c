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

/*IRQ handler*/
static irqreturn_t kmb_isr(int irq, void *arg);

static struct clk *clk_lcd;
static struct clk *clk_mipi;

static int kmb_display_clk_enable(void)
{
	clk_prepare_enable(clk_lcd);
	clk_prepare_enable(clk_mipi);
	return 0;
}

static int kmb_display_clk_disable(void)
{
	if (clk_lcd)
		clk_disable_unprepare(clk_lcd);
	if (clk_mipi)
		clk_disable_unprepare(clk_mipi);
	return 0;
}

static int kmb_load(struct drm_device *drm, unsigned long flags)
{
	struct kmb_drm_private *dev_p = drm->dev_private;
	struct platform_device *pdev = to_platform_device(drm->dev);
	struct drm_bridge *bridge;
	/*struct resource *res;*/
	/*u32 version;*/
	int irq_lcd, irq_mipi;
	int ret;
	struct device_node *encoder_node;

	/* TBD - not sure if clock_get needs to be called here */
	/*
	 *dev_p->clk = devm_clk_get(drm->dev, "pxlclk");
	 *if (IS_ERR(dev_p->clk))
	 *	return PTR_ERR(dev_p->clk);
	 */
	/*
	 * TBD call this in the future when device tree is ready,
	 * use hardcoded value for now
	 */
	/*res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	 *dev_p->lcd_mmio = devm_ioremap_resource(drm->dev, res);
	 *
	 *if (IS_ERR(dev_p->lcd_mmio)) {
	 *	DRM_ERROR("failed to map control registers area\n");
	 *	ret = PTR_ERR(dev_p->lcd_mmio);
	 *	dev_p->lcd_mmio = NULL;
	 *	return ret;
	 *}
	 */
	 /* LCD mmio */
	if (!request_mem_region(LCD_BASE_ADDR, LCD_MMIO_SIZE, "kmb-lcd")) {
		DRM_ERROR("failed to reserve LCD registers\n");
		return -ENOMEM;
	}
	dev_p->lcd_mmio = ioremap_nocache(LCD_BASE_ADDR, LCD_MMIO_SIZE);
	if (!dev_p->lcd_mmio) {
		DRM_ERROR("failed to map LCD registers\n");
		return -ENOMEM;
	}

	/* Mipi mmio */
	if (!request_mem_region(MIPI_BASE_ADDR, MIPI_MMIO_SIZE, "kmb-mipi")) {
		DRM_ERROR("failed to reserve MIPI registers\n");
		iounmap(dev_p->lcd_mmio);
		return -ENOMEM;
	}
	dev_p->mipi_mmio = ioremap_nocache(MIPI_BASE_ADDR, MIPI_MMIO_SIZE);
	if (!dev_p->mipi_mmio) {
		DRM_ERROR("failed to map MIPI registers\n");
		iounmap(dev_p->lcd_mmio);
		return -ENOMEM;
	}

	/*this is only for MIPI_TX_MSS_LCD_MIPI_CFG register */
	dev_p->msscam_mmio = ioremap_nocache(MSS_CAM_BASE_ADDR,
			MSS_CAM_MMIO_SIZE);

	/* register irqs here - section 17.3 in databook
	 * lists LCD at 79 and 82 for MIPI under MSS CPU -
	 * firmware has to redirect it to A53
	 */
	irq_lcd = platform_get_irq_byname(pdev, "irq_lcd");
	if (irq_lcd < 0) {
		DRM_ERROR("irq_lcd not found");
		return irq_lcd;
	}
	pr_info("irq_lcd platform_get_irq = %d\n", irq_lcd);
	ret = request_irq(irq_lcd, kmb_isr, IRQF_SHARED, "irq_lcd", dev_p);
	dev_p->irq_lcd = irq_lcd;

	irq_mipi = platform_get_irq_byname(pdev, "irq_mipi");
	if (irq_mipi < 0) {
		DRM_ERROR("irq_mipi not found");
		return irq_mipi;
	}
	pr_info("irq_mipi platform_get_irq = %d\n", irq_mipi);
	ret = request_irq(irq_mipi, kmb_isr, IRQF_SHARED, "irq_mipi", dev_p);
	dev_p->irq_mipi = irq_mipi;



/*TBD read and check for correct product version here */

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

	/* find ADV7535 node and initialize it */
	encoder_node = of_parse_phandle(drm->dev->of_node, "encoder-slave", 0);
	if (!encoder_node) {
		DRM_ERROR("failed to get bridge info from DT\n");
		ret = -EPROBE_DEFER;
		goto setup_fail;
	}

	/* Locate drm bridge from the hdmi encoder DT node */
	bridge = of_drm_find_bridge(encoder_node);
	if (!bridge) {
		DRM_ERROR("failed to get bridge driver from DT\n");
		ret = -EPROBE_DEFER;
		goto setup_fail;
	}

	of_node_put(encoder_node);

	ret = kmb_dsi_init(drm, bridge);
	if (ret) {
		DRM_ERROR("failed to initialize DSI\n");
		goto setup_fail;
	}

	/* enable display clocks*/
	clk_lcd = clk_get(&pdev->dev, "clk_lcd");
	if (!clk_lcd) {
		DRM_ERROR("clk_get() failed clk_lcd\n");
		goto setup_fail;
	}
	clk_mipi = clk_get(&pdev->dev, "clk_mipi");
	if (!clk_mipi) {
		DRM_ERROR("clk_get() failed clk_mipi\n");
		goto setup_fail;
	}
	kmb_display_clk_enable();

	ret = drm_irq_install(drm, platform_get_irq(pdev, 0));
	if (ret < 0) {
		DRM_ERROR("failed to install IRQ handler\n");
		goto irq_fail;
	}

	return 0;

irq_fail:
	drm_crtc_cleanup(&dev_p->crtc);
setup_fail:
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
	.name = "kmb",
	.desc = "KEEMBAY DISPLAY DRIVER ",
	.date = "20190122",
	.major = 1,
	.minor = 0,
};

static int kmb_drm_bind(struct device *dev)
{
	struct drm_device *drm;
	struct kmb_drm_private *lcd;
	int ret;

	drm = drm_dev_alloc(&kmb_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	lcd = devm_kzalloc(dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	drm->dev_private = lcd;
	dev_set_drvdata(dev, drm);

	kmb_setup_mode_config(drm);
	ret = kmb_load(drm, 0);
	if (ret)
		goto err_free;

	/* Set the CRTC's port so that the encoder component can find it */
	lcd->crtc.port = of_graph_get_port_by_id(dev->of_node, 0);

	ret = component_bind_all(dev, drm);
	if (ret) {
		DRM_ERROR("Failed to bind all components\n");
		goto err_unload;
	}

	ret = pm_runtime_set_active(dev);
	if (ret)
		goto err_pm_active;

	pm_runtime_enable(dev);

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret < 0) {
		DRM_ERROR("failed to initialise vblank\n");
		goto err_vblank;
	}

	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);

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
err_pm_active:
	component_unbind_all(dev, drm);
err_unload:
	of_node_put(lcd->crtc.port);
	lcd->crtc.port = NULL;
	drm_irq_uninstall(drm);
	of_reserved_mem_device_release(drm->dev);
err_free:
	drm_mode_config_cleanup(drm);
	dev_set_drvdata(dev, NULL);
	drm_dev_put(drm);

	return ret;
}

static void kmb_drm_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct kmb_drm_private *dev_p = drm->dev_private;

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	component_unbind_all(dev, drm);
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
}

static const struct component_master_ops kmb_master_ops = {
	.bind = kmb_drm_bind,
	.unbind = kmb_drm_unbind,
};

static int compare_dev(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int kmb_probe(struct platform_device *pdev)
{
	struct device_node *port;
	struct component_match *match = NULL;

	/* there is only one output port inside each device, find it */
	port = of_graph_get_remote_node(pdev->dev.of_node, 0, 0);
	if (!port)
		return -ENODEV;

	drm_of_component_match_add(&pdev->dev, &match, compare_dev, port);
	of_node_put(port);

	return component_master_add_with_match(&pdev->dev, &kmb_master_ops,
					       match);
}

static int kmb_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &kmb_master_ops);
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
	.probe = kmb_probe,
	.remove = kmb_remove,
	.driver = {
		   .name = "Keembay_Display",
		   .pm = &kmb_pm_ops,
		   .of_match_table = kmb_of_match,
		   },
};

module_platform_driver(kmb_platform_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Keembay Display driver");
MODULE_LICENSE("GPL v2");
