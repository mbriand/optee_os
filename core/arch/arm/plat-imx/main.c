/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * All rights reserved.
 * Copyright (c) 2016, Wind River Systems.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arm32.h>
#include <console.h>
#include <drivers/gic.h>
#include <drivers/imx_uart.h>
#include <io.h>
#include <kernel/generic_boot.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/pm_stubs.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <stdint.h>
#include <sm/optee_smc.h>
#include <tee/entry_fast.h>
#include <tee/entry_std.h>

#if defined(PLATFORM_FLAVOR_mx6qsabrelite) || \
	defined(PLATFORM_FLAVOR_mx6qsabresd) || \
	defined(PLATFORM_FLAVOR_mx6dlsabresd)
#include <kernel/tz_ssvce_pl310.h>
#endif

static void main_fiq(void);
static struct gic_data gic_data;

static const struct thread_handlers handlers = {
	.std_smc = tee_entry_std,
	.fast_smc = tee_entry_fast,
	.nintr = main_fiq,
	.cpu_on = pm_panic,
	.cpu_off = pm_panic,
	.cpu_suspend = pm_panic,
	.cpu_resume = pm_panic,
	.system_off = pm_panic,
	.system_reset = pm_panic,
};

static struct imx_uart_data console_data __early_bss;

register_phys_mem(MEM_AREA_IO_NSEC, CONSOLE_UART_BASE, CORE_MMU_DEVICE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, GIC_BASE, CORE_MMU_DEVICE_SIZE);

#if defined(PLATFORM_FLAVOR_mx6qsabrelite) || \
	defined(PLATFORM_FLAVOR_mx6qsabresd) || \
	defined(PLATFORM_FLAVOR_mx6dlsabresd)
register_phys_mem(MEM_AREA_IO_SEC, PL310_BASE, CORE_MMU_DEVICE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, SRC_BASE, CORE_MMU_DEVICE_SIZE);
#endif

const struct thread_handlers *generic_boot_get_handlers(void)
{
	return &handlers;
}

static void main_fiq(void)
{
	panic();
}

#if defined(PLATFORM_FLAVOR_mx6qsabrelite) || \
	defined(PLATFORM_FLAVOR_mx6qsabresd) || \
	defined(PLATFORM_FLAVOR_mx6dlsabresd)

static inline int log2_(unsigned int val)
{
	unsigned int r = 0;

	while (val >>= 1)
		r++;

	return r;
}

static int set_tzasc_config(void)
{
	uint32_t ram_mode;
	uint32_t tzasc_enabled;

	tzasc_enabled = read32(IOMUXC_BASE + IOMUXC_GPR9)
		& IOMUXC_GPR9_TZASC_MASK;
	if (!tzasc_enabled) {
		IMSG("TZASC is not enabled, please flash corresponding fuse.");
		/* TZASC can be enabled by flashing TZASC_ENABLE using u-boot
		 * command fuse prog 0 6 10000000
		 */
		return 1;
	}

	ram_mode = read32(SRC_BASE + SRC_SBMR1) & SRC_SBMR1_DDR_MASK;
	if (ram_mode == SRC_SBMR1_DDR_SINGLE) {
		const uint32_t ram_end = 0x50000000;
		uint32_t start;
		uint32_t size;

		/* On access violation, issue a DECERR but no interrupt */
		write32(TZASC_ACT_LOW_DECERR, TZASC1_BASE + TZASC_ACT);

		/* Region 0 covers all RAM and is accessible by anyone */
		write32(TZASC_REGATTR_ALLOW_ALL,
				TZASC1_BASE + TZASC_REGATTR(0));

		/* Region 1 covers TEE RAM and is accessible only by the secure
		 * world. This region partly overlaps region 0 and overrides
		 * access rights set previously on this zone.
		 */
#ifdef CFG_WITH_PAGER
		start = TZSRAM_BASE;
#else
		start = CFG_DDR_TEETZ_RESERVED_START;
#endif
		size = ram_end - start;

		if ((size & (size - 1)) != 0) {
			IMSG("TEE reserved RAM must be a power of 2.");
			return 1;
		}

		write32(start, TZASC1_BASE + TZASC_REGLOW(1));
		write32(0, TZASC1_BASE + TZASC_REGHIGH(1));
		write32(TZASC_REGATTR_ALLOW_SECURE
				| TZASC_REGATTR_SIZE(log2_(size))
				| TZASC_REGATTR_EN,
				TZASC1_BASE + TZASC_REGATTR(1));

		/* Region 2 covers TEE shared memory and is accessible by both
		 * worlds.  This region partly overlaps region 1 and overrides
		 * access rights set previously on this zone.
		 */
		start = CFG_SHMEM_START;
		size = ram_end - start;

		if ((size & (size - 1)) != 0) {
			IMSG("TEE shared RAM must be a power of 2.");
			return 1;
		}

		write32(start, TZASC1_BASE + TZASC_REGLOW(2));
		write32(0, TZASC1_BASE + TZASC_REGHIGH(2));
		write32(TZASC_REGATTR_ALLOW_ALL
				| TZASC_REGATTR_SIZE(log2_(size))
				| TZASC_REGATTR_EN,
				TZASC1_BASE + TZASC_REGATTR(2));
	} else {
		IMSG("Unsupported DDR mode, skipping TZASC configuration.");
		return 1;
	}

	return 0;
}

void plat_cpu_reset_late(void)
{
	uintptr_t addr;

	if (!get_core_pos()) {
		int ret;

		/* primary core */
#if defined(CFG_BOOT_SYNC_CPU)
		/* set secondary entry address and release core */
		write32(CFG_TEE_LOAD_ADDR, SRC_BASE + SRC_GPR1 + 8);
		write32(CFG_TEE_LOAD_ADDR, SRC_BASE + SRC_GPR1 + 16);
		write32(CFG_TEE_LOAD_ADDR, SRC_BASE + SRC_GPR1 + 24);

		write32(SRC_SCR_CPU_ENABLE_ALL, SRC_BASE + SRC_SCR);
#endif

		/* SCU config */
		write32(SCU_INV_CTRL_INIT, SCU_BASE + SCU_INV_SEC);
		write32(SCU_SAC_CTRL_INIT, SCU_BASE + SCU_SAC);
		write32(SCU_NSAC_CTRL_INIT, SCU_BASE + SCU_NSAC);

		/* SCU enable */
		write32(read32(SCU_BASE + SCU_CTRL) | 0x1,
			SCU_BASE + SCU_CTRL);

		/* configure imx6 CSU */

		/* first grant all peripherals */
		for (addr = CSU_BASE + CSU_CSL_START;
			 addr != CSU_BASE + CSU_CSL_END;
			 addr += 4)
			write32(CSU_ACCESS_ALL, addr);

		/* Lock TZASC access from Non-Secure world */
		write32(CSU_ACCESS_SECURE, CSU_BASE + CSU_CSL16);

		/* lock the settings */
		for (addr = CSU_BASE + CSU_CSL_START;
			 addr != CSU_BASE + CSU_CSL_END;
			 addr += 4)
			write32(read32(addr) | CSU_SETTING_LOCK, addr);

		ret = set_tzasc_config();
		if (ret != 0)
			EMSG("TZASC not configured, TEE memory is not secure.");
	}
}
#endif

void console_init(void)
{
	imx_uart_init(&console_data, CONSOLE_UART_BASE);
	register_serial_console(&console_data.chip);
}

void main_init_gic(void)
{
	vaddr_t gicc_base;
	vaddr_t gicd_base;

	gicc_base = (vaddr_t)phys_to_virt(GIC_BASE + GICC_OFFSET,
					  MEM_AREA_IO_SEC);
	gicd_base = (vaddr_t)phys_to_virt(GIC_BASE + GICD_OFFSET,
					  MEM_AREA_IO_SEC);

	if (!gicc_base || !gicd_base)
		panic();

	/* Initialize GIC */
	gic_init(&gic_data, gicc_base, gicd_base);
	itr_init(&gic_data.chip);
}

#if defined(PLATFORM_FLAVOR_mx6qsabrelite) || \
	defined(PLATFORM_FLAVOR_mx6qsabresd) || \
	defined(PLATFORM_FLAVOR_mx6dlsabresd)
vaddr_t pl310_base(void)
{
	static void *va __early_bss;

	if (cpu_mmu_enabled()) {
		if (!va)
			va = phys_to_virt(PL310_BASE, MEM_AREA_IO_SEC);
		return (vaddr_t)va;
	}
	return PL310_BASE;
}

void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);
}
#endif
