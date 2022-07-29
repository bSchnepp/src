/* $NetBSD: bcm2835_cprman.c,v 1.5 2021/01/27 03:10:19 thorpej Exp $ */

/*-
 * Copyright (c) 2017 Jared D. McNeill <jmcneill@invisible.ca>
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: bcm2835_cprman.c,v 1.5 2021/01/27 03:10:19 thorpej Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/bus.h>

#include <dev/clk/clk_backend.h>

#include <dev/fdt/fdtvar.h>

#include <arm/broadcom/bcm2835var.h>
#include <evbarm/rpi/vcprop.h>

enum {
	RPI_POWER_DOMAIN_USB = 3,
	RPI_POWER_DOMAIN_V3D = 10,
	POWER_NCLOCK
};

struct power_domain_packet {
	u_int		domain;
	u_int		enabled;
};

struct power_softc {
	device_t	sc_dev;
	int		sc_phandle;
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "raspberrypi,bcm2835-power" },
	DEVICE_COMPAT_EOL
};

static int
power_match(device_t parent, cfdata_t cf, void *aux)
{
	const struct fdt_attach_args *faa = aux;
	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
power_attach(device_t parent, device_t self, void *aux)
{
	struct power_softc * const sc = device_private(self);
	const struct fdt_attach_args *faa = aux;
	const int phandle = faa->faa_phandle;

	sc->sc_dev = self;
	sc->sc_phandle = phandle;

	aprint_naive("\n");
	aprint_normal(": BCM283x Power Controller\n");
}

CFATTACH_DECL_NEW(bcmpower_fdt, sizeof(struct power_softc),
    power_match, power_attach, NULL, NULL);
