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

#include <drivers/gic.h>
#include <drivers/tzc380.h>
#include <io.h>
#include <kernel/generic_boot.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/tz_ssvce_pl310.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <platform_config.h>

register_phys_mem(MEM_AREA_IO_SEC, PL310_BASE, CORE_MMU_DEVICE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, SRC_BASE, CORE_MMU_DEVICE_SIZE);


#if defined(CFG_TZC380)
static void set_tzasc_config(void)
{
	uint32_t ram_mode;
	uint32_t tzasc_enabled;
	int ret;

	tzasc_enabled = read32(IOMUXC_BASE + IOMUXC_GPR9)
		& IOMUXC_GPR9_TZASC_MASK;
	if (!tzasc_enabled) {
		IMSG("TZASC is not enabled, please flash corresponding fuse.");
		/* TZASC can be enabled by flashing TZASC_ENABLE using u-boot
		 * command "fuse prog 0 6 10000000"
		 */
		return;
	}

	ram_mode = read32(SRC_BASE + SRC_SBMR1) & SRC_SBMR1_DDR_MASK;
	if (ram_mode != SRC_SBMR1_DDR_SINGLE) {
		IMSG("Unsupported DDR mode, skipping TZASC configuration.");
		return;
	}

	ret = tzc380_init(TZASC1_BASE);
	if (ret != TZC380_OK)
		panic("TZASC initialization failed !");

	/* Region 0 covers all RAM and is accessible by anyone. */
	ret = tzc380_set_default_region(TZC380_REGATTR_ALLOW_ALL);
	if (ret != TZC380_OK)
		panic("TZASC Region 0 setting failed !");

	/* Region 1 covers TEE RAM and is accessible only by the secure
	 * world. This region partly overlaps region 0 and overrides
	 * access rights set previously on this zone.
	 */
	ret = tzc380_add_region(TZDRAM_BASE, TZDRAM_SIZE + CFG_SHMEM_SIZE,
				TZC380_REGATTR_ALLOW_SECURE, 0);
	if (ret != TZC380_OK)
		panic("TZASC Region 1 setting failed !");

	/* Region 2 covers TEE shared memory and is accessible by both
	 * worlds.  This region partly overlaps region 1 and overrides
	 * access rights set previously on this zone.
	 */
	ret = tzc380_add_region(CFG_SHMEM_START, CFG_SHMEM_SIZE,
				TZC380_REGATTR_ALLOW_ALL, 0);
	if (ret != TZC380_OK)
		panic("TZASC Region 2 setting failed !");
}
#endif

void plat_cpu_reset_late(void)
{
	uintptr_t addr;

	if (!get_core_pos()) {
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

#if defined(CFG_TZC380)
		/* Lock TZASC access from Non-Secure world */
		write32(CSU_ACCESS_SECURE, CSU_BASE + CSU_CSL16);
#endif

		/* lock the settings */
		for (addr = CSU_BASE + CSU_CSL_START;
			 addr != CSU_BASE + CSU_CSL_END;
			 addr += 4)
			write32(read32(addr) | CSU_SETTING_LOCK, addr);

#if defined(CFG_TZC380)
		set_tzasc_config();
#endif
	}
}

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

