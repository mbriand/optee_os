/*
 * Copyright (c) 2017, Witekio.
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

#include <drivers/tzc380.h>
#include <io.h>
#include <platform_config.h>
#include <trace.h>

struct tzc380 {
	vaddr_t base;
	unsigned int max_regions;
	unsigned int last_region;
};

static struct tzc380 tzc380_dev;

static inline int log2_(unsigned int val)
{
	unsigned int r = 0;

	while (val >>= 1)
		r++;

	return r;
}

/* TZC380 registers helper functions.
 */

static void tzc380_write_region_start_low(unsigned int id, uint32_t val)
{
	/* ID 0 always covers all the RAM area. */
	if (id != 0)
		write32(val, tzc380_dev.base + TZC380_REGLOW(id));
}

static void tzc380_write_region_start_high(unsigned int id, uint32_t val)
{
	/* ID 0 always covers all the RAM area. */
	if (id != 0)
		write32(val, tzc380_dev.base + TZC380_REGHIGH(id));
}

static void tzc380_write_region_attr(unsigned int id, uint32_t val)
{
	write32(val, tzc380_dev.base + TZC380_REGATTR(id));
}

static uint8_t tzc380_get_max_regions(void)
{
	return read32(tzc380_dev.base + TZC380_CONF) & TZC380_CONF_REGNO;
}

static uint32_t tzc380_get_component_id(void)
{
	uint32_t val = 0;
	unsigned int i;
	paddr_t reg = tzc380_dev.base + TZC380_COMPONENT_ID;

	for (i = 0; i < 4; ++i) {
		val |= (read32(reg) & 0xFF) << i * 8;
		reg += sizeof(uint32_t);
	}

	return val;
}

static void tzc380_set_action(uint32_t action)
{
	write32(action, tzc380_dev.base + TZC380_ACT);
}

/* Set TZC380 default region covering all RAM area.
 */
enum tzc380_result tzc380_set_default_region(uint32_t perm)
{
	tzc380_write_region_attr(0, perm & TZC380_REGATTR_PERM_MASK);

	return TZC380_OK;
}

/* Add a TZC380 region. Region size must always be a power of 2.
 */
enum tzc380_result tzc380_add_region(paddr_t start, paddr_t size, uint32_t perm,
				     uint8_t disable_mask)
{
	unsigned int id;

	if ((tzc380_dev.last_region + 1) > tzc380_dev.max_regions)
		return TZC380_ERR_NO_AVAIL_REG;

	if ((size & (size - 1)) != 0) {
		IMSG("TZC380 region size must be a power of 2.");
		return TZC380_ERR_INVALID_PARAM;
	}

	id = ++tzc380_dev.last_region;
	tzc380_write_region_start_low(id, start & 0xFFFFFFFF);
#ifdef ARM64
	tzc380_write_region_start_high(id, (start >> 32) & 0xFFFFFFFF);
#else
	tzc380_write_region_start_high(id, 0);
#endif
	disable_mask = (disable_mask << TZC380_REGATTR_DIS_SHIFT)
		& TZC380_REGATTR_DIS_MASK;
	tzc380_write_region_attr(id, (perm & TZC380_REGATTR_PERM_MASK)
				 | disable_mask
				 | TZC380_REGATTR_SIZE(log2_(size))
				 | TZC380_REGATTR_EN);

	return TZC380_OK;
}

/* TZC380 driver initialization.
 */
enum tzc380_result tzc380_init(paddr_t base)
{
	tzc380_dev.base = base;

	/* Ensure we are talking to a TZC380 device. */
	if (tzc380_get_component_id() != TZC380_COMPONENT_ID_VAL) {
		return TZC380_ERR_NOT_FOUND;
	}

	/* Get module capabilities. */
	tzc380_dev.max_regions = tzc380_get_max_regions();
	tzc380_dev.last_region = 0;

	/* On access violation, issue a DECERR but no interrupt. */
	tzc380_set_action(TZC380_ACT_LOW_DECERR);


	return TZC380_OK;
}

