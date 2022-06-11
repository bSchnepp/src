/*	$NetBSD: $	*/

/*-
 * Copyright (c) 2018 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Taylor R. Campbell.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: $");

#include <sys/types.h>
#include <sys/module.h>
#ifndef _MODULE
#include <sys/once.h>
#endif
#include <sys/systm.h>

#include <drm/drmP.h>
#include <drm/drm_sysctl.h>

#include "lima_amdkfd.h"
#include "lima_drv.h"

MODULE(MODULE_CLASS_DRIVER, lima, "drmkms"); /* XXX drmkms_i2c, drmkms_ttm */

#ifdef _MODULE
#include "ioconf.c"
#endif

/* XXX Kludge to get these from lima_drv.c.  */
extern struct drm_driver *const lima_drm_driver;
extern int lima_max_kms_ioctl;

struct drm_sysctl_def lima_def = DRM_SYSCTL_INIT();

static int
lima_init(void)
{
	int error;

	error = drm_guarantee_initialized();
	if (error)
		return error;

	lima_drm_driver->num_ioctls = lima_max_kms_ioctl;
	lima_drm_driver->driver_features |= DRIVER_MODESET;

#if notyet			/* XXX lima acpi */
	lima_register_atpx_handler();
#endif

	lima_amdkfd_init();
	drm_sysctl_init(&lima_def);

	return 0;
}

int	lima_guarantee_initialized(void); /* XXX */
int
lima_guarantee_initialized(void)
{
#ifdef _MODULE
	return 0;
#else
	static ONCE_DECL(lima_init_once);

	return RUN_ONCE(&lima_init_once, &lima_init);
#endif
}

static void
lima_fini(void)
{

	drm_sysctl_fini(&lima_def);
	lima_amdkfd_fini();
#if notyet			/* XXX lima acpi */
	lima_unregister_atpx_handler();
#endif
}

static int
lima_modcmd(modcmd_t cmd, void *arg __unused)
{
	int error;

	switch (cmd) {
	case MODULE_CMD_INIT:
		/* XXX Kludge it up...  Must happen before attachment.  */
#ifdef _MODULE
		error = lima_init();
#else
		error = lima_guarantee_initialized();
#endif
		if (error) {
			aprint_error("lima: failed to initialize: %d\n",
			    error);
			return error;
		}
#ifdef _MODULE
		error = config_init_component(cfdriver_ioconf_lima,
		    cfattach_ioconf_lima, cfdata_ioconf_lima);
		if (error) {
			aprint_error("lima: failed to init component"
			    ": %d\n", error);
			lima_fini();
			return error;
		}
#endif
		return 0;

	case MODULE_CMD_FINI:
#ifdef _MODULE
		error = config_fini_component(cfdriver_ioconf_lima,
		    cfattach_ioconf_lima, cfdata_ioconf_lima);
		if (error) {
			aprint_error("lima: failed to fini component"
			    ": %d\n", error);
			return error;
		}
#endif
		lima_fini();
		return 0;

	default:
		return ENOTTY;
	}
}
