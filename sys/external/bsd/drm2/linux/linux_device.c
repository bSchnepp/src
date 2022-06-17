/*	$NetBSD$	*/

/*-
 * Copyright (c) 2022 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Robert Swindells.
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
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/device.h>

#include <dev/fdt/fdtvar.h>

#include <drm/drm_device.h>

#include <linux/device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>

struct gpu_softc {
	device_t		sc_dev;
	struct drm_device	*sc_drm_dev;
	void			*sc_pdev;
	int			sc_phandle;
};

struct reset_control {
	struct fdtbus_reset rc_reset;
};

struct clk *devm_clk_get(struct device *dev, const char *id)
{
	struct gpu_softc *const sc = device_private(dev);
	struct clk *fdtclk;

	fdtclk = fdtbus_clock_get(sc->sc_phandle, id);
	if (!fdtclk)
		printf("couldn't get clock %s\n", id);
	return fdtclk;

}

int clk_prepare_enable(struct clk *clk)
{

	if (clk_enable(clk) != 0) {
		printf("couldn't enable clock\n");
		return -1;
	}
	return 0;
}

void clk_disable_unprepare(struct clk *clk)
{

	clk_disable(clk);
}

struct reset_control *
devm_reset_control_array_get_optional_shared(struct device *dev)
{
	struct gpu_softc *const sc = device_private(dev);

	return (struct reset_control *) fdtbus_reset_get_index(sc->sc_phandle, 0);
}

int reset_control_assert(struct reset_control *reset)
{

	return -fdtbus_reset_assert((struct fdtbus_reset *) reset);;
}

int reset_control_deassert(struct reset_control *reset)
{

	return -fdtbus_reset_deassert((struct fdtbus_reset *) reset);
}

int devm_request_irq(struct device *dev, unsigned int irq,
    irq_handler_t handler, unsigned long irqflags,
    const char *devname, void *dev_id)
{
	struct gpu_softc *const sc = device_private(dev);
	char intrstr[128];

	if (!fdtbus_intr_str(sc->sc_phandle, 0,
		intrstr, sizeof(intrstr))) {
		aprint_error_dev(dev, "failed to decode interrupt\n");
		return -1;
	}

	sc->sc_drm_dev->irq_cookie = fdtbus_intr_establish_xname(sc->sc_phandle, 0, IPL_BIO,
	    0, handler, dev_id, devname);
	
	return 0;
}

struct regulator *
devm_regulator_get_optional(struct device *dev, const char *id)
{
	struct gpu_softc *const sc = device_private(dev);
	struct fdtbus_regulator *reg;

	reg = fdtbus_regulator_acquire(sc->sc_phandle, id);
	printf("devm_regulator_get_optional: %s %p\n", id, reg);
	return (struct regulator *) reg;
}

void *dma_alloc_wc(struct device *dev, size_t size,
				bus_dmamap_t *dma_addr, int gfp)
{
	struct gpu_softc *const sc = device_private(dev);
	void *addr;
	int error;

	error = bus_dmamap_create(sc->sc_drm_dev->dmat, size, 1, size, 0,
		BUS_DMA_WAITOK, dma_addr);
	if (error)
		goto fail0;
	error = bus_dmamem_alloc(sc->sc_drm_dev->dmat, size, sizeof(uint32_t),
	    0, (*dma_addr)->dm_segs, 1, &(*dma_addr)->dm_nsegs, BUS_DMA_WAITOK);
	if (error)
		goto fail1;
	error = bus_dmamem_map(sc->sc_drm_dev->dmat, (*dma_addr)->dm_segs,
	    (*dma_addr)->dm_nsegs, size, &addr, BUS_DMA_WAITOK | BUS_DMA_COHERENT);
	if (error)
		goto fail2;
	
	return addr;

fail2:
	bus_dmamem_free(sc->sc_drm_dev->dmat,
	    (*dma_addr)->dm_segs, (*dma_addr)->dm_nsegs);
fail1:
	bus_dmamap_destroy(sc->sc_drm_dev->dmat, *dma_addr);
fail0:
	return (void *) 0;
}

void dma_free_wc(struct device *dev, size_t size,
				void *cpu_addr, bus_dmamap_t dma_addr)
{
	struct gpu_softc *const sc = device_private(dev);

	bus_dmamem_unmap(sc->sc_drm_dev->dmat, cpu_addr, size);
	bus_dmamap_destroy(sc->sc_drm_dev->dmat, dma_addr);
}

void *dev_get_drvdata(struct device *dev)
{
	return device_private(dev);
}

