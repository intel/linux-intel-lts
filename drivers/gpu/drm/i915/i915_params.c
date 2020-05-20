/*
 * Copyright © 2014 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <drm/drm_print.h>

#include "i915_params.h"
#include "i915_drv.h"

#define i915_param_named(name, T, perm, desc) \
	module_param_named(name, i915_modparams.name, T, perm); \
	MODULE_PARM_DESC(name, desc)
#define i915_param_named_unsafe(name, T, perm, desc) \
	module_param_named_unsafe(name, i915_modparams.name, T, perm); \
	MODULE_PARM_DESC(name, desc)

struct i915_params i915_modparams __read_mostly = {
#define MEMBER(T, member, value) .member = (value),
	I915_PARAMS_FOR_EACH(MEMBER)
#undef MEMBER
};

i915_param_named(modeset, int, 0400,
	"Use kernel modesetting [KMS] (0=disable, "
	"1=on, -1=force vga console preference [default])");

i915_param_named_unsafe(enable_dc, int, 0400,
	"Enable power-saving display C-states. "
	"(-1=auto [default]; 0=disable; 1=up to DC5; 2=up to DC6; "
	"3=up to DC5 with DC3CO; 4=up to DC6 with DC3CO)");

i915_param_named_unsafe(enable_fbc, int, 0600,
	"Enable frame buffer compression for power savings "
	"(default: -1 (use per-chip default))");

i915_param_named_unsafe(lvds_channel_mode, int, 0400,
	 "Specify LVDS channel mode "
	 "(0=probe BIOS [default], 1=single-channel, 2=dual-channel)");

i915_param_named_unsafe(panel_use_ssc, int, 0600,
	"Use Spread Spectrum Clock with panels [LVDS/eDP] "
	"(default: auto from VBT)");

i915_param_named_unsafe(vbt_sdvo_panel_type, int, 0400,
	"Override/Ignore selection of SDVO panel mode in the VBT "
	"(-2=ignore, -1=auto [default], index in VBT BIOS table)");

i915_param_named_unsafe(reset, int, 0600,
	"Attempt GPU resets (0=disabled, 1=full gpu reset, 2=engine reset [default])");

i915_param_named_unsafe(vbt_firmware, charp, 0400,
	"Load VBT from specified file under /lib/firmware");

#if IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR)
i915_param_named(error_capture, bool, 0600,
	"Record the GPU state following a hang. "
	"This information in /sys/class/drm/card<N>/error is vital for "
	"triaging and debugging hangs.");
#endif

i915_param_named_unsafe(enable_hangcheck, bool, 0600,
	"Periodically check GPU activity for detecting hangs. "
	"WARNING: Disabling this can cause system wide hangs. "
	"(default: true)");

i915_param_named_unsafe(enable_psr, int, 0600,
	"Enable PSR "
	"(0=disabled, 1=enabled) "
	"Default: -1 (use per-chip default)");

i915_param_named(psr_safest_params, bool, 0400,
	"Replace PSR VBT parameters by the safest and not optimal ones. This "
	"is helpfull to detect if PSR issues are related to bad values set in "
	" VBT. (0=use VBT paramters, 1=use safest parameters)");

i915_param_named_unsafe(force_probe, charp, 0400,
	"Force probe the driver for specified devices. "
	"See CONFIG_DRM_I915_FORCE_PROBE for details.");

i915_param_named_unsafe(alpha_support, bool, 0400,
	"Deprecated. See i915.force_probe.");

i915_param_named_unsafe(disable_power_well, int, 0400,
	"Disable display power wells when possible "
	"(-1=auto [default], 0=power wells always on, 1=power wells disabled when possible)");

i915_param_named_unsafe(enable_ips, int, 0600, "Enable IPS (default: true)");

i915_param_named(fastboot, int, 0600,
	"Try to skip unnecessary mode sets at boot time "
	"(0=disabled, 1=enabled) "
	"Default: -1 (use per-chip default)");

i915_param_named_unsafe(prefault_disable, bool, 0600,
	"Disable page prefaulting for pread/pwrite/reloc (default:false). "
	"For developers only.");

i915_param_named_unsafe(load_detect_test, bool, 0600,
	"Force-enable the VGA load detect code for testing (default:false). "
	"For developers only.");

i915_param_named_unsafe(force_reset_modeset_test, bool, 0600,
	"Force a modeset during gpu reset for testing (default:false). "
	"For developers only.");

i915_param_named_unsafe(invert_brightness, int, 0600,
	"Invert backlight brightness "
	"(-1 force normal, 0 machine defaults, 1 force inversion), please "
	"report PCI device ID, subsystem vendor and subsystem device ID "
	"to dri-devel@lists.freedesktop.org, if your machine needs it. "
	"It will then be included in an upcoming module version.");

i915_param_named(disable_display, bool, 0400,
	"Disable display (default: false)");

i915_param_named(mmio_debug, int, 0600,
	"Enable the MMIO debug code for the first N failures (default: off). "
	"This may negatively affect performance.");

i915_param_named(verbose_state_checks, bool, 0600,
	"Enable verbose logs (ie. WARN_ON()) in case of unexpected hw state conditions.");

i915_param_named_unsafe(nuclear_pageflip, bool, 0400,
	"Force enable atomic functionality on platforms that don't have full support yet.");

/* WA to get away with the default setting in VBT for early platforms.Will be removed */
i915_param_named_unsafe(edp_vswing, int, 0400,
	"Ignore/Override vswing pre-emph table selection from VBT "
	"(0=use value from vbt [default], 1=low power swing(200mV),"
	"2=default swing(400mV))");

i915_param_named_unsafe(enable_guc, int, 0400,
	"Enable GuC load for GuC submission and/or HuC load. "
	"Required functionality can be selected using bitmask values. "
	"(-1=auto, 0=disable [default], 1=GuC submission, 2=HuC load)");

i915_param_named(guc_log_level, int, 0400,
	"GuC firmware logging level. Requires GuC to be loaded. "
	"(-1=auto [default], 0=disable, 1..4=enable with verbosity min..max)");

i915_param_named_unsafe(guc_firmware_path, charp, 0400,
	"GuC firmware path to use instead of the default one");

i915_param_named_unsafe(huc_firmware_path, charp, 0400,
	"HuC firmware path to use instead of the default one");

i915_param_named_unsafe(dmc_firmware_path, charp, 0400,
	"DMC firmware path to use instead of the default one");

i915_param_named_unsafe(enable_dp_mst, bool, 0600,
	"Enable multi-stream transport (MST) for new DisplayPort sinks. (default: true)");

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG)
i915_param_named_unsafe(inject_probe_failure, uint, 0400,
	"Force an error after a number of failure check points (0:disabled (default), N:force failure at the Nth failure check point)");
#endif

i915_param_named(enable_dpcd_backlight, int, 0600,
	"Enable support for DPCD backlight control"
	"(-1=use per-VBT LFP backlight type setting, 0=disabled [default], 1=enabled)");

#if IS_ENABLED(CONFIG_DRM_I915_GVT)
i915_param_named(enable_gvt, bool, 0400,
	"Enable support for Intel GVT-g graphics virtualization host support(default:false)");
#endif

#if IS_ENABLED(CONFIG_DRM_I915_UNSTABLE_FAKE_LMEM)
i915_param_named_unsafe(fake_lmem_start, ulong, 0600,
	"Fake LMEM start offset (default: 0)");
#endif

/* pipeA Scaler = BITS 0-7 pipeB scaler = 8-15, pipeC = 16-19
 *
 * +----------+------------+-------------+------------+
 * |unused    |  Pipe C    |   Pipe B    |   Pipe A   |
 * +----------+------------+-------------+------------+
 * 31       20 19        16 15           8 7           0
 *
 * Each nibble represents domain id. 0 for Dom0, 1,2,3...0xF for DomUs
 * eg: domains_scaler_owners = 0x00030210 // 0x000|3|02|10
 * scaler          domain
 * scaler_owner1A -0
 * scaler_owner2A -1
 * scaler_owner1B -2
 * scaler_owner2B -0
 * scaler_owner1C -3
 * scaler_owner2C -0
 *
 */
i915_param_named(domain_scaler_owner, int, 0400,
	"scaler owners for each domain and for each pipe ids can be from 0-F");

i915_param_named(enable_pvmmio, uint, 0400,
	"Enable pv mmio feature and set pvmmio level, default 1."
	"This parameter could only set from host, guest value is set through vgt_if");

i915_param_named(gvt_workload_priority, int, 0600,
		"Set GVT-g workload priority, (range: (-1023, 1023), default: 0, "
		"more positive value means higher priority).");

/* pipeA = BITS 0-3, pipeB = BITS 8-11, pipeC = BITS 16-18
 * +----------+-------+---------+--------+--------+--------+--------+
 * |unused    |unused |  Pipe C | unused | Pipe B | unused | Pipe A |
 * +----------+-------+---------+--------+--------+--------+--------+
 * 31         23      18        15       11       7        3        0
 *
 *
 * BITS 0,1,2,3 - needs to be set planes assigned for pipes A and B
 * and BITs 0,1,2 - for pipe C
 * eg: avail_planes_per_pipe = 0x3 - pipe A=2(planes 1 and 2) , pipeB=0 and pipeC=0 planes
 * eg: avail_planes_per_pipe = 0x5 - pipe A=2(planes 1 and 3) , pipeB=0 and pipeC=0 planes
 * avail_planes_per_pipe = 0x030701 - pipe A =1(plane 1, pipeB=3(planes 1,2 and 3), pipeC=2( planes 1 and 2)
 *
 */
i915_param_named_unsafe(avail_planes_per_pipe, ulong, 0400,
	"plane mask for each	pipe: \
	set BITS 0-3:pipeA 8-11:pipeB 16-18:pipeC to specify the planes that \
	are available eg: 0x030701 : planes 1:pipeA 1,2,3:pipeB \
	1,2:pipeC (0x0 - default value)");

/* pipeA = BITS 0-15 pipeB = 16-31, pipeC = 32-47
 *
 * +----------+------------+-------------+------------+
 * |unused    |  Pipe C    |   Pipe B    |   Pipe A   |
 * +----------+------------+-------------+------------+
 * 63         47           31            15           0
 *
 * Each nibble represents domain id. 0 for Dom0, 1,2,3...0xF for DomUs
 * eg: domain_plane_owners = 0x022111000010 // 0x0221|1100|0010
 * plane		 domain
 * plane_owner1A -0
 * plane_owner2A -1
 * plane_owner3A -0
 * plane_owner4A -0
 * plane_owner1B -0
 * plane_owner2B -0
 * plane_owner3B -1
 * plane_owner4B -1
 * plane_owner1C -1
 * plane_owner2C -2
 * plane_owner3C -2
 *
 *
 */
i915_param_named_unsafe(domain_plane_owners, ulong, 0400,
	"plane owners for each domain and for each pipe \
	ids can be from 0-F,  eg: domain_plane_owners = 0x022111000010 \
	planes owner: 3C:2 2C:2 1C:1 4B:1 3B:1 2B:1 1B:0 4A:0 3A:0 2A:1 1A:0 \
	(0x0 - default value)");

i915_param_named_unsafe(enable_context_restore, bool, 0400,
	"To togger non-context regs save/restore feature(default:false)");

i915_param_named(enable_hp_work, bool, 0400,
	"To enable active the high-performance mode during vGPU busy(default:false)");

static __always_inline void _print_param(struct drm_printer *p,
					 const char *name,
					 const char *type,
					 const void *x)
{
	if (!__builtin_strcmp(type, "bool"))
		drm_printf(p, "i915.%s=%s\n", name, yesno(*(const bool *)x));
	else if (!__builtin_strcmp(type, "int"))
		drm_printf(p, "i915.%s=%d\n", name, *(const int *)x);
	else if (!__builtin_strcmp(type, "unsigned int"))
		drm_printf(p, "i915.%s=%u\n", name, *(const unsigned int *)x);
	else if (!__builtin_strcmp(type, "unsigned long"))
		drm_printf(p, "i915.%s=%lu\n", name, *(const unsigned long *)x);
	else if (!__builtin_strcmp(type, "char *"))
		drm_printf(p, "i915.%s=%s\n", name, *(const char **)x);
	else
		WARN_ONCE(1, "no printer defined for param type %s (i915.%s)\n",
			  type, name);
}

/**
 * i915_params_dump - dump i915 modparams
 * @params: i915 modparams
 * @p: the &drm_printer
 *
 * Pretty printer for i915 modparams.
 */
void i915_params_dump(const struct i915_params *params, struct drm_printer *p)
{
#define PRINT(T, x, ...) _print_param(p, #x, #T, &params->x);
	I915_PARAMS_FOR_EACH(PRINT);
#undef PRINT
}

static __always_inline void dup_param(const char *type, void *x)
{
	if (!__builtin_strcmp(type, "char *"))
		*(void **)x = kstrdup(*(void **)x, GFP_ATOMIC);
}

void i915_params_copy(struct i915_params *dest, const struct i915_params *src)
{
	*dest = *src;
#define DUP(T, x, ...) dup_param(#T, &dest->x);
	I915_PARAMS_FOR_EACH(DUP);
#undef DUP
}

static __always_inline void free_param(const char *type, void *x)
{
	if (!__builtin_strcmp(type, "char *")) {
		kfree(*(void **)x);
		*(void **)x = NULL;
	}
}

/* free the allocated members, *not* the passed in params itself */
void i915_params_free(struct i915_params *params)
{
#define FREE(T, x, ...) free_param(#T, &params->x);
	I915_PARAMS_FOR_EACH(FREE);
#undef FREE
}
