/* $NetBSD: $ */

/*-
 * Copyright (c) 2022 Robert Swindells <rjs@fdy2.co.uk>
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
__KERNEL_RCSID(0, "$NetBSD: $");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/endian.h>
#include <sys/errno.h>
#include <sys/mbuf.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/mutex.h>
#include <sys/uio.h>

#include <dev/fdt/fdtvar.h>

#include <drm/drm_device.h>
#include <drm/drm_drv.h>

#include <lima_drv.h>

static int lima_match(device_t, cfdata_t, void *);
static void lima_attach(device_t, device_t, void *);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "arm,mali-400",
	  .data = NULL },
	{ .compat = "arm,mali-450",
	  .data = NULL },
	DEVICE_COMPAT_EOL
};

struct lima_softc {
	device_t		sc_dev;
	struct drm_device	*sc_drm_dev;
	struct lima_device	sc_ldev;
};

#if 0
struct lima_device *
lima_device_private(device_t self)
{
	struct lima_softc *const sc = device_private(self);

	return sc->sc_drm_dev->dev_private;
}
#endif

CFATTACH_DECL_NEW(lima, sizeof(struct lima_softc),
	lima_match, lima_attach, NULL, NULL);

/* XXX Kludge to get these from lima_drv.c.  */
extern struct drm_driver *const lima_driver;

static int
lima_match(device_t parent, cfdata_t cfdata, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
lima_attach(device_t parent, device_t self, void *aux)
{
	struct lima_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	sc->sc_dev = self;

	sc->sc_drm_dev = drm_dev_alloc(lima_driver, self);
	if (IS_ERR(sc->sc_drm_dev)) {
		aprint_error_dev(self, "unable to create drm device: %ld\n",
		    PTR_ERR(sc->sc_drm_dev));
		sc->sc_drm_dev = NULL;
		return;
	}
	sc->sc_drm_dev->dev_private = &sc->sc_ldev;
	sc->sc_ldev.dev = sc->sc_dev;
	sc->sc_ldev.ddev = sc->sc_drm_dev;
	sc->sc_ldev.phandle = faa->faa_phandle;

	sc->sc_drm_dev->bst = faa->faa_bst;
	sc->sc_drm_dev->dmat = faa->faa_dmat;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	printf("lima_attach: size %lx\n", size);
	error = bus_space_map(faa->faa_bst, addr, size, 0, &sc->sc_ldev.bsh);
	if (error) {
		aprint_error(": failed to map register %#lx@%#lx: %d\n",
		    size, addr, error);
		return;
	}

	/* XXX errno Linux->NetBSD */
	error = -lima_device_init(&sc->sc_ldev);
	if (error) {
		aprint_error(": couldn't init device %d\n", error);
		return;
	}

	/* XXX errno Linux->NetBSD */
	error = -drm_dev_register(sc->sc_drm_dev, 0);
	if (error) {
		aprint_error_dev(self, "unable to register drm: %d\n", error);
		return;
	}

	aprint_naive("\n");
	aprint_normal(": GPU\n");

}
