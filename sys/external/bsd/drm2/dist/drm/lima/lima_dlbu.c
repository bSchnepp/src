/*	$NetBSD$	*/

// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright 2018-2019 Qiang Yu <yuq825@gmail.com> */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <linux/io.h>
#include <linux/device.h>

#include "lima_device.h"
#include "lima_dlbu.h"
#include "lima_vm.h"
#include "lima_regs.h"

#ifdef __NetBSD__
#define dlbu_write(reg, data) bus_space_write_4(ip->bst, ip->bsh, (reg), (data))
#define dlbu_read(reg) bus_space_read_4(ip->bst, ip->bsh, (reg))
#else
#define dlbu_write(reg, data) writel(data, ip->iomem + reg)
#define dlbu_read(reg) readl(ip->iomem + reg)
#endif

void lima_dlbu_enable(struct lima_device *dev, int num_pp)
{
	struct lima_sched_pipe *pipe = dev->pipe + lima_pipe_pp;
	struct lima_ip *ip = dev->ip + lima_ip_dlbu;
	int i, mask = 0;

	for (i = 0; i < num_pp; i++) {
		struct lima_ip *pp = pipe->processor[i];

		mask |= 1 << (pp->id - lima_ip_pp0);
	}

	dlbu_write(LIMA_DLBU_PP_ENABLE_MASK, mask);
}

void lima_dlbu_disable(struct lima_device *dev)
{
	struct lima_ip *ip = dev->ip + lima_ip_dlbu;

	dlbu_write(LIMA_DLBU_PP_ENABLE_MASK, 0);
}

void lima_dlbu_set_reg(struct lima_ip *ip, u32 *reg)
{
	dlbu_write(LIMA_DLBU_TLLIST_VBASEADDR, reg[0]);
	dlbu_write(LIMA_DLBU_FB_DIM, reg[1]);
	dlbu_write(LIMA_DLBU_TLLIST_CONF, reg[2]);
	dlbu_write(LIMA_DLBU_START_TILE_POS, reg[3]);
}

int lima_dlbu_init(struct lima_ip *ip)
{
	struct lima_device *dev = ip->dev;

#ifdef __NetBSD__
	dlbu_write(LIMA_DLBU_MASTER_TLLIST_PHYS_ADDR,
		dev->dlbu_dma->dm_segs[0].ds_addr | 1);
#else
	dlbu_write(LIMA_DLBU_MASTER_TLLIST_PHYS_ADDR, dev->dlbu_dma | 1);
#endif
	dlbu_write(LIMA_DLBU_MASTER_TLLIST_VADDR, LIMA_VA_RESERVE_DLBU);

	return 0;
}

void lima_dlbu_fini(struct lima_ip *ip)
{

}
