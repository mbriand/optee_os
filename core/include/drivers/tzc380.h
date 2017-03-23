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

#ifndef __DRIVERS_TZC380_H
#define __DRIVERS_TZC380_H

#include <types_ext.h>
#include <util.h>

#define TZC380_CONF			0x00
#define TZC380_ACT			0x04
#define TZC380_REGLOW_0			0x100
#define TZC380_REGHIGH_0		0x104
#define TZC380_REGATTR_0		0x108
#define TZC380_REGLOW(reg)		(TZC380_REGLOW_0 + 0x10 * reg)
#define TZC380_REGHIGH(reg)		(TZC380_REGHIGH_0 + 0x10 * reg)
#define TZC380_REGATTR(reg)		(TZC380_REGATTR_0 + 0x10 * reg)
#define TZC380_COMPONENT_ID		0xFF0

#define TZC380_CONF_REGNO		(0xF)
#define TZC380_ACT_LOW_DECERR		0x01
#define TZC380_REGATTR_PERM_MASK	SHIFT_U32(0xF, 28)
#define TZC380_REGATTR_ALLOW_ALL	SHIFT_U32(0xF, 28)
#define TZC380_REGATTR_ALLOW_SECURE	SHIFT_U32(0xC, 28)
#define TZC380_REGATTR_SIZE(sz_log)	SHIFT_U32(sz_log - 1, 1)
#define TZC380_REGATTR_DIS_SHIFT	8
#define TZC380_REGATTR_DIS_MASK		SHIFT_U32(0xFF, TZC380_REGATTR_DIS_SHIFT)
#define TZC380_REGATTR_EN		0x01
#define TZC380_COMPONENT_ID_VAL		0xB105F00D

enum tzc380_result {
	TZC380_OK,
	TZC380_ERR_NOT_FOUND,
	TZC380_ERR_INVALID_PARAM,
	TZC380_ERR_NO_AVAIL_REG,
};

enum tzc380_result tzc380_set_default_region(uint32_t perm);
enum tzc380_result tzc380_add_region(paddr_t start, paddr_t size,
				     uint32_t perm, uint8_t disable_mask);
enum tzc380_result tzc380_init(paddr_t base);

#endif /* __DRIVERS_TZC380_H */
