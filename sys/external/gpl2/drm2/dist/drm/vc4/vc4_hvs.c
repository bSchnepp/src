/*	$NetBSD$	*/

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Broadcom
 */

/**
 * DOC: VC4 HVS module.
 *
 * The Hardware Video Scaler (HVS) is the piece of hardware that does
 * translation, scaling, colorspace conversion, and compositing of
 * pixels stored in framebuffers into a FIFO of pixels going out to
 * the Pixel Valve (CRTC).  It operates at the system clock rate (the
 * system audio clock gate, specifically), which is much higher than
 * the pixel clock rate.
 *
 * There is a single global HVS, with multiple output FIFOs that can
 * be consumed by the PVs.  This file just manages the resources for
 * the HVS, while the vc4_crtc.c code actually drives HVS setup for
 * each CRTC.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#ifdef __NetBSD__
#include <dev/fdt/fdtvar.h>
#endif

#include <linux/component.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>

#include "vc4_drv.h"
#include "vc4_regs.h"

#ifndef __NetBSD__
static const struct debugfs_reg32 hvs_regs[] = {
	VC4_REG32(SCALER_DISPCTRL),
	VC4_REG32(SCALER_DISPSTAT),
	VC4_REG32(SCALER_DISPID),
	VC4_REG32(SCALER_DISPECTRL),
	VC4_REG32(SCALER_DISPPROF),
	VC4_REG32(SCALER_DISPDITHER),
	VC4_REG32(SCALER_DISPEOLN),
	VC4_REG32(SCALER_DISPLIST0),
	VC4_REG32(SCALER_DISPLIST1),
	VC4_REG32(SCALER_DISPLIST2),
	VC4_REG32(SCALER_DISPLSTAT),
	VC4_REG32(SCALER_DISPLACT0),
	VC4_REG32(SCALER_DISPLACT1),
	VC4_REG32(SCALER_DISPLACT2),
	VC4_REG32(SCALER_DISPCTRL0),
	VC4_REG32(SCALER_DISPBKGND0),
	VC4_REG32(SCALER_DISPSTAT0),
	VC4_REG32(SCALER_DISPBASE0),
	VC4_REG32(SCALER_DISPCTRL1),
	VC4_REG32(SCALER_DISPBKGND1),
	VC4_REG32(SCALER_DISPSTAT1),
	VC4_REG32(SCALER_DISPBASE1),
	VC4_REG32(SCALER_DISPCTRL2),
	VC4_REG32(SCALER_DISPBKGND2),
	VC4_REG32(SCALER_DISPSTAT2),
	VC4_REG32(SCALER_DISPBASE2),
	VC4_REG32(SCALER_DISPALPHA2),
	VC4_REG32(SCALER_OLEDOFFS),
	VC4_REG32(SCALER_OLEDCOEF0),
	VC4_REG32(SCALER_OLEDCOEF1),
	VC4_REG32(SCALER_OLEDCOEF2),
};
#endif

void vc4_hvs_dump_state(struct drm_device *dev)
{
#ifdef __NetBSD__
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	int i;
#else
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	struct drm_printer p = drm_info_printer(&vc4->hvs->pdev->dev);
	int i;

	drm_print_regset32(&p, &vc4->hvs->regset);
#endif

	DRM_INFO("HVS ctx:\n");
	for (i = 0; i < 64; i += 4) {
#ifdef __NetBSD__
		DRM_INFO("0x%08x (%s): 0x%08x 0x%08x 0x%08x 0x%08x\n",
			 i * 4, i < HVS_BOOTLOADER_DLIST_END ? "B" : "D",
			 bus_space_read_4(vc4->hvs->dlist_bst, 
			 	vc4->hvs->dlist_bsh, i + 0),
			 bus_space_read_4(vc4->hvs->dlist_bst, 
			 	vc4->hvs->dlist_bsh, i + 1),
			 bus_space_read_4(vc4->hvs->dlist_bst, 
			 	vc4->hvs->dlist_bsh, i + 2),
			 bus_space_read_4(vc4->hvs->dlist_bst, 
			 	vc4->hvs->dlist_bsh, i + 3));
#else
		DRM_INFO("0x%08x (%s): 0x%08x 0x%08x 0x%08x 0x%08x\n",
			 i * 4, i < HVS_BOOTLOADER_DLIST_END ? "B" : "D",
			 readl((u32 __iomem *)vc4->hvs->dlist + i + 0),
			 readl((u32 __iomem *)vc4->hvs->dlist + i + 1),
			 readl((u32 __iomem *)vc4->hvs->dlist + i + 2),
			 readl((u32 __iomem *)vc4->hvs->dlist + i + 3));
#endif
	}
}

#ifndef __NetBSD__
static int vc4_hvs_debugfs_underrun(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	struct drm_printer p = drm_seq_file_printer(m);

	drm_printf(&p, "%d\n", atomic_read(&vc4->underrun));

	return 0;
}
#endif

/* The filter kernel is composed of dwords each containing 3 9-bit
 * signed integers packed next to each other.
 */
#define VC4_INT_TO_COEFF(coeff) (coeff & 0x1ff)
#define VC4_PPF_FILTER_WORD(c0, c1, c2)				\
	((((c0) & 0x1ff) << 0) |				\
	 (((c1) & 0x1ff) << 9) |				\
	 (((c2) & 0x1ff) << 18))

/* The whole filter kernel is arranged as the coefficients 0-16 going
 * up, then a pad, then 17-31 going down and reversed within the
 * dwords.  This means that a linear phase kernel (where it's
 * symmetrical at the boundary between 15 and 16) has the last 5
 * dwords matching the first 5, but reversed.
 */
#define VC4_LINEAR_PHASE_KERNEL(c0, c1, c2, c3, c4, c5, c6, c7, c8,	\
				c9, c10, c11, c12, c13, c14, c15)	\
	{VC4_PPF_FILTER_WORD(c0, c1, c2),				\
	 VC4_PPF_FILTER_WORD(c3, c4, c5),				\
	 VC4_PPF_FILTER_WORD(c6, c7, c8),				\
	 VC4_PPF_FILTER_WORD(c9, c10, c11),				\
	 VC4_PPF_FILTER_WORD(c12, c13, c14),				\
	 VC4_PPF_FILTER_WORD(c15, c15, 0)}

#define VC4_LINEAR_PHASE_KERNEL_DWORDS 6
#define VC4_KERNEL_DWORDS (VC4_LINEAR_PHASE_KERNEL_DWORDS * 2 - 1)

/* Recommended B=1/3, C=1/3 filter choice from Mitchell/Netravali.
 * http://www.cs.utexas.edu/~fussell/courses/cs384g/lectures/mitchell/Mitchell.pdf
 */
static const u32 mitchell_netravali_1_3_1_3_kernel[] =
	VC4_LINEAR_PHASE_KERNEL(0, -2, -6, -8, -10, -8, -3, 2, 18,
				50, 82, 119, 155, 187, 213, 227);

static int vc4_hvs_upload_linear_kernel(struct vc4_hvs *hvs,
					struct drm_mm_node *space,
					const u32 *kernel)
{
	int ret, i;

#ifndef __NetBSD__
	u32 __iomem *dst_kernel;
#endif

	ret = drm_mm_insert_node(&hvs->dlist_mm, space, VC4_KERNEL_DWORDS);
	if (ret) {
		DRM_ERROR("Failed to allocate space for filter kernel: %d\n",
			  ret);
		return ret;
	}

#ifdef __NetBSD__
	for (i = 0; i < VC4_KERNEL_DWORDS; i++) {
		if (i < VC4_LINEAR_PHASE_KERNEL_DWORDS)
			bus_space_write_4(hvs->dlist_bst, hvs->dlist_bsh, 
				space->start + i, kernel[i]);
		else {
			bus_space_write_4(hvs->dlist_bst, hvs->dlist_bsh, 
				space->start + i, 
				kernel[VC4_KERNEL_DWORDS - i - 1]);
		}
	}
#else
	dst_kernel = hvs->dlist + space->start;

	for (i = 0; i < VC4_KERNEL_DWORDS; i++) {
		if (i < VC4_LINEAR_PHASE_KERNEL_DWORDS)
			writel(kernel[i], &dst_kernel[i]);
		else {
			writel(kernel[VC4_KERNEL_DWORDS - i - 1],
			       &dst_kernel[i]);
		}
	}
#endif

	return 0;
}

void vc4_hvs_mask_underrun(struct drm_device *dev, int channel)
{
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	u32 dispctrl = HVS_READ(SCALER_DISPCTRL);

	dispctrl &= ~SCALER_DISPCTRL_DSPEISLUR(channel);

	HVS_WRITE(SCALER_DISPCTRL, dispctrl);
}

void vc4_hvs_unmask_underrun(struct drm_device *dev, int channel)
{
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	u32 dispctrl = HVS_READ(SCALER_DISPCTRL);

	dispctrl |= SCALER_DISPCTRL_DSPEISLUR(channel);

	HVS_WRITE(SCALER_DISPSTAT,
		  SCALER_DISPSTAT_EUFLOW(channel));
	HVS_WRITE(SCALER_DISPCTRL, dispctrl);
}


static void vc4_hvs_report_underrun(struct drm_device *dev)
{
	struct vc4_dev *vc4 = to_vc4_dev(dev);

	atomic_inc(&vc4->underrun);
	DRM_DEV_ERROR(dev->dev, "HVS underrun\n");
}

#ifdef __NetBSD__
static irqreturn_t vc4_hvs_irq_handler(void *data)
#else
static irqreturn_t vc4_hvs_irq_handler(int irq, void *data)
#endif
{
	struct drm_device *dev = data;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	irqreturn_t irqret = IRQ_NONE;
	int channel;
	u32 control;
	u32 status;

	status = HVS_READ(SCALER_DISPSTAT);
	control = HVS_READ(SCALER_DISPCTRL);

	for (channel = 0; channel < SCALER_CHANNELS_COUNT; channel++) {
		/* Interrupt masking is not always honored, so check it here. */
		if (status & SCALER_DISPSTAT_EUFLOW(channel) &&
		    control & SCALER_DISPCTRL_DSPEISLUR(channel)) {
			vc4_hvs_mask_underrun(dev, channel);
			vc4_hvs_report_underrun(dev);

			irqret = IRQ_HANDLED;
		}
	}

	/* Clear every per-channel interrupt flag. */
	HVS_WRITE(SCALER_DISPSTAT, SCALER_DISPSTAT_IRQMASK(0) |
				   SCALER_DISPSTAT_IRQMASK(1) |
				   SCALER_DISPSTAT_IRQMASK(2));

	return irqret;
}

#if __NetBSD__

static int vc4hvs_match(device_t, cfdata_t, void *);
static void vc4hvs_attach(device_t, device_t, void *);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "brcm,bcm2835-hvs" },
	DEVICE_COMPAT_EOL
};

struct vcfourhvs_softc {
	device_t		sc_dev;
	struct drm_device	*sc_drm_dev;
	void			*sc_pdev;
	int			sc_phandle;
	struct vc4_hvs		sc_hvs;
	void			*sc_ih;
};

CFATTACH_DECL_NEW(vcfourhvs, sizeof(struct vcfourhvs_softc),
	vc4hvs_match, vc4hvs_attach, NULL, NULL);

/* XXX Kludge to get these from vc4_drv.c.  */
extern struct drm_device *vc4_drm_device;

static int
vc4hvs_match(device_t parent, cfdata_t cfdata, void *aux)
{
	struct fdt_attach_args * const faa = aux;
	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
vc4hvs_attach(device_t parent, device_t self, void *aux)
{
	struct vcfourhvs_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;

	struct vc4_hvs * hvs = NULL;
	struct vc4_dev * vc4 = NULL;

	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	uint32_t dispcfg = 0;

	sc->sc_dev = self;
	sc->sc_drm_dev = vc4_drm_device;
	sc->sc_drm_dev->bst = faa->faa_bst;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_phandle = faa->faa_phandle;

	error = bus_space_map(faa->faa_bst, addr, size, 0, &hvs->bsh);
	if (error) {
		aprint_error(": failed to map register %#lx@%#lx: %d\n",
		    size, addr, error);
		return;
	}

	/* 31 registers, based on the size of the array hvs_regs. */
	hvs->dlist_bst = hvs->bst;
	error = bus_space_subregion(hvs->bst, hvs->bsh, SCALER_DLIST_START, 
		31 * sizeof(uint32_t), &hvs->dlist_bsh);
	if (IS_ERR(hvs->bst)) {
		aprint_error_dev(self, "unable to map regs region: %d\n", 
			EINVAL);
		return;		
	}

	spin_lock_init(&hvs->mm_lock);

	/* Set up the dlist, and avoid changing things the bootloader set up
	 * in the process. The Linux driver does this to prevent strange
	 * visual issues.
	 */
	drm_mm_init(&hvs->dlist_mm,
		    HVS_BOOTLOADER_DLIST_END,
		    (SCALER_DLIST_SIZE >> 2) - HVS_BOOTLOADER_DLIST_END);
	drm_mm_init(&hvs->lbm_mm, 0, 96 * 1024);

	error = vc4_hvs_upload_linear_kernel(hvs,
					   &hvs->mitchell_netravali_filter,
					   mitchell_netravali_1_3_1_3_kernel);

	dispcfg = HVS_READ(SCALER_DISPCTRL);

	dispcfg |= SCALER_DISPCTRL_ENABLE;

	dispcfg |= SCALER_DISPCTRL_DISPEIRQ(0) |
		    SCALER_DISPCTRL_DISPEIRQ(1) |
		    SCALER_DISPCTRL_DISPEIRQ(2);

	
	/* Similarly do as the Linux driver does: set up channel 2 for usage. */
	dispcfg &= ~SCALER_DISPCTRL_DSP3_MUX_MASK;

	dispcfg &= ~(SCALER_DISPCTRL_DMAEIRQ |
		      SCALER_DISPCTRL_SLVWREIRQ |
		      SCALER_DISPCTRL_SLVRDEIRQ |
		      SCALER_DISPCTRL_SCLEIRQ);

	dispcfg &= ~(SCALER_DISPCTRL_DSPEIEOF(0) |
		      SCALER_DISPCTRL_DSPEIEOF(1) |
		      SCALER_DISPCTRL_DSPEIEOF(2));

	dispcfg &= ~(SCALER_DISPCTRL_DSPEIEOLN(0) |
		      SCALER_DISPCTRL_DSPEIEOLN(1) |
		      SCALER_DISPCTRL_DSPEIEOLN(2));

	dispcfg &= ~(SCALER_DISPCTRL_DSPEISLUR(0) |
		      SCALER_DISPCTRL_DSPEISLUR(1) |
		      SCALER_DISPCTRL_DSPEISLUR(2));

	dispcfg |= VC4_SET_FIELD(2, SCALER_DISPCTRL_DSP3_MUX);
	HVS_WRITE(SCALER_DISPCTRL, dispcfg);	

	sc->sc_ih = fdtbus_intr_establish(phandle, 0, IPL_VM, IST_LEVEL,
			       vc4_hvs_irq_handler, &sc->sc_hvs);
	if (sc->sc_ih == NULL) {
		aprint_error_dev(self, "unable to register irq: %d\n", error);
		return;
	}

	aprint_naive("\n");
	aprint_normal(": GPU\n");
}

#else

static int vc4_hvs_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = dev_get_drvdata(master);
	struct vc4_dev *vc4 = drm->dev_private;
	struct vc4_hvs *hvs = NULL;
	int ret;
	u32 dispctrl;

	hvs = devm_kzalloc(&pdev->dev, sizeof(*hvs), GFP_KERNEL);
	if (!hvs)
		return -ENOMEM;

	hvs->pdev = pdev;

	hvs->regs = vc4_ioremap_regs(pdev, 0);
	if (IS_ERR(hvs->regs))
		return PTR_ERR(hvs->regs);

	hvs->regset.base = hvs->regs;
	hvs->regset.regs = hvs_regs;
	hvs->regset.nregs = ARRAY_SIZE(hvs_regs);

	hvs->dlist = hvs->regs + SCALER_DLIST_START;

	spin_lock_init(&hvs->mm_lock);

	/* Set up the HVS display list memory manager.  We never
	 * overwrite the setup from the bootloader (just 128b out of
	 * our 16K), since we don't want to scramble the screen when
	 * transitioning from the firmware's boot setup to runtime.
	 */
	drm_mm_init(&hvs->dlist_mm,
		    HVS_BOOTLOADER_DLIST_END,
		    (SCALER_DLIST_SIZE >> 2) - HVS_BOOTLOADER_DLIST_END);

	/* Set up the HVS LBM memory manager.  We could have some more
	 * complicated data structure that allowed reuse of LBM areas
	 * between planes when they don't overlap on the screen, but
	 * for now we just allocate globally.
	 */
	drm_mm_init(&hvs->lbm_mm, 0, 96 * 1024);

	/* Upload filter kernels.  We only have the one for now, so we
	 * keep it around for the lifetime of the driver.
	 */
	ret = vc4_hvs_upload_linear_kernel(hvs,
					   &hvs->mitchell_netravali_filter,
					   mitchell_netravali_1_3_1_3_kernel);
	if (ret)
		return ret;

	vc4->hvs = hvs;

	dispctrl = HVS_READ(SCALER_DISPCTRL);

	dispctrl |= SCALER_DISPCTRL_ENABLE;
	dispctrl |= SCALER_DISPCTRL_DISPEIRQ(0) |
		    SCALER_DISPCTRL_DISPEIRQ(1) |
		    SCALER_DISPCTRL_DISPEIRQ(2);

	/* Set DSP3 (PV1) to use HVS channel 2, which would otherwise
	 * be unused.
	 */
	dispctrl &= ~SCALER_DISPCTRL_DSP3_MUX_MASK;
	dispctrl &= ~(SCALER_DISPCTRL_DMAEIRQ |
		      SCALER_DISPCTRL_SLVWREIRQ |
		      SCALER_DISPCTRL_SLVRDEIRQ |
		      SCALER_DISPCTRL_DSPEIEOF(0) |
		      SCALER_DISPCTRL_DSPEIEOF(1) |
		      SCALER_DISPCTRL_DSPEIEOF(2) |
		      SCALER_DISPCTRL_DSPEIEOLN(0) |
		      SCALER_DISPCTRL_DSPEIEOLN(1) |
		      SCALER_DISPCTRL_DSPEIEOLN(2) |
		      SCALER_DISPCTRL_DSPEISLUR(0) |
		      SCALER_DISPCTRL_DSPEISLUR(1) |
		      SCALER_DISPCTRL_DSPEISLUR(2) |
		      SCALER_DISPCTRL_SCLEIRQ);
	dispctrl |= VC4_SET_FIELD(2, SCALER_DISPCTRL_DSP3_MUX);

	HVS_WRITE(SCALER_DISPCTRL, dispctrl);

	ret = devm_request_irq(dev, platform_get_irq(pdev, 0),
			       vc4_hvs_irq_handler, 0, "vc4 hvs", drm);
	if (ret)
		return ret;

	vc4_debugfs_add_regset32(drm, "hvs_regs", &hvs->regset);
	vc4_debugfs_add_file(drm, "hvs_underrun", vc4_hvs_debugfs_underrun,
			     NULL);

	return 0;
}

static void vc4_hvs_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct drm_device *drm = dev_get_drvdata(master);
	struct vc4_dev *vc4 = drm->dev_private;

	if (drm_mm_node_allocated(&vc4->hvs->mitchell_netravali_filter))
		drm_mm_remove_node(&vc4->hvs->mitchell_netravali_filter);

	drm_mm_takedown(&vc4->hvs->dlist_mm);
	drm_mm_takedown(&vc4->hvs->lbm_mm);

	vc4->hvs = NULL;
}


static const struct component_ops vc4_hvs_ops = {
	.bind   = vc4_hvs_bind,
	.unbind = vc4_hvs_unbind,
};

static int vc4_hvs_dev_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &vc4_hvs_ops);
}

static int vc4_hvs_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vc4_hvs_ops);
	return 0;
}

static const struct of_device_id vc4_hvs_dt_match[] = {
	{ .compatible = "brcm,bcm2835-hvs" },
	{}
};

struct platform_driver vc4_hvs_driver = {
	.probe = vc4_hvs_dev_probe,
	.remove = vc4_hvs_dev_remove,
	.driver = {
		.name = "vc4_hvs",
		.of_match_table = vc4_hvs_dt_match,
	},
};

#endif