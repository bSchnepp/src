/*	$NetBSD$	*/

// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright 2017-2019 Qiang Yu <yuq825@gmail.com> */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <linux/iopoll.h>
#include <linux/device.h>

#include "lima_device.h"
#include "lima_pmu.h"
#include "lima_regs.h"

#ifdef __NetBSD__
#define pmu_write(reg, data) bus_space_write_4(ip->bst, ip->bsh, reg, data)
#define pmu_read(reg) bus_space_read_4(ip->bst, ip->bsh, reg)
#else
#define pmu_write(reg, data) writel(data, ip->iomem + reg)
#define pmu_read(reg) readl(ip->iomem + reg)
#endif

static int lima_pmu_wait_cmd(struct lima_ip *ip)
{
	struct lima_device *dev = ip->dev;
	int err;
	u32 v;

#ifdef __NetBSD__
	err = readl_poll_timeout(ip->bst, ip->bsh, LIMA_PMU_INT_RAWSTAT,
				 v, v & LIMA_PMU_INT_CMD_MASK,
				 100, 100000);
#else
	err = readl_poll_timeout(ip->iomem + LIMA_PMU_INT_RAWSTAT,
				 v, v & LIMA_PMU_INT_CMD_MASK,
				 100, 100000);
#endif
	if (err) {
		dev_err(dev->dev, "timeout wait pmd cmd\n");
		return err;
	}

	pmu_write(LIMA_PMU_INT_CLEAR, LIMA_PMU_INT_CMD_MASK);
	return 0;
}

int lima_pmu_init(struct lima_ip *ip)
{
	int err;
	u32 stat;

	pmu_write(LIMA_PMU_INT_MASK, 0);

	/* If this value is too low, when in high GPU clk freq,
	 * GPU will be in unstable state.
	 */
	pmu_write(LIMA_PMU_SW_DELAY, 0xffff);

	/* status reg 1=off 0=on */
	stat = pmu_read(LIMA_PMU_STATUS);

	/* power up all ip */
	if (stat) {
		pmu_write(LIMA_PMU_POWER_UP, stat);
		err = lima_pmu_wait_cmd(ip);
		if (err)
			return err;
	}
	return 0;
}

void lima_pmu_fini(struct lima_ip *ip)
{

}
