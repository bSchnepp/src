/*	$NetBSD: $	*/

/*-
 * Copyright (c) 2021 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Robert Swindells
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

#ifndef _LINUX_IOPOLL_H_
#define _LINUX_IOPOLL_H_

#include <sys/types.h>
#include <sys/bus.h>

#include <linux/delay.h>
#include <linux/ktime.h>

#define readl_poll_timeout(bst, bsh, addr, val, cond, sleep_us, timeout_us) \
({ \
	u64 __timeout_us = (timeout_us); \
	unsigned long __sleep_us = (sleep_us); \
	ktime_t __timeout = ktime_get() + __timeout_us; \
	bus_space_barrier((bst), (bsh), (addr), 4, BUS_SPACE_BARRIER_READ); \
	for (;;) { \
		(val) = bus_space_read_4((bst), (bsh), (addr)); \
		if (cond) \
			break; \
		if (__timeout_us && \
		    (ktime_get() - __timeout) > 0) { \
			(val) = bus_space_read_4((bst), (bsh), (addr)); \
			break; \
		} \
		if (__sleep_us) \
			usleep_range((__sleep_us >> 2) + 1, __sleep_us); \
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})

#define readl_poll_timeout_atomic(bst, bsh, addr, val, cond, delay_us, timeout_us) \
({ \
	u64 __timeout_us = (timeout_us); \
	unsigned long __delay_us = (delay_us); \
	ktime_t __timeout = ktime_get() + __timeout_us; \
	bus_space_barrier((bst), (bsh), (addr), 4, BUS_SPACE_BARRIER_READ); \
	for (;;) { \
		(val) = bus_space_read_4((bst), (bsh), (addr)); \
		if (cond) \
			break; \
		if (__timeout_us && \
		    (ktime_get() - __timeout) > 0) { \
		(val) = bus_space_read_4((bst), (bsh), (addr)); \
		break; \
		} \
		if (__delay_us) \
			udelay(__delay_us); \
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})

#define readl_relaxed_poll_timeout(bst, bsh, addr, val, cond, sleep_us, timeout_us) \
({ \
	u64 __timeout_us = (timeout_us); \
	unsigned long __sleep_us = (sleep_us); \
	ktime_t __timeout = ktime_get() + __timeout_us; \
	for (;;) { \
		(val) = bus_space_read_4((bst), (bsh), (addr)); \
		if (cond) \
			break; \
		if (__timeout_us && \
		    (ktime_get() - __timeout) > 0) { \
			(val) = bus_space_read_4((bst), (bsh), (addr)); \
			break; \
		} \
		if (__sleep_us) \
			usleep_range((__sleep_us >> 2) + 1, __sleep_us); \
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})

#define readl_relaxed_poll_timeout_atomic(bst, bsh, addr, val, cond, delay_us, timeout_us) \
({ \
	u64 __timeout_us = (timeout_us); \
	unsigned long __delay_us = (delay_us); \
	ktime_t __timeout = ktime_get() + __timeout_us; \
	for (;;) { \
		(val) = bus_space_read_4((bst), (bsh), (addr)); \
		if (cond) \
			break; \
		if (__timeout_us && \
		    (ktime_get() - __timeout) > 0) { \
		(val) = bus_space_read_4((bst), (bsh), (addr)); \
		break; \
		} \
		if (__delay_us) \
			udelay(__delay_us); \
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})


#endif  /* _LINUX_IOPOLL_H_ */
