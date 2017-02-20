/*
 * ALSA SoC ES7242 codec driver
 *
 * Author:      David Yang, <yangxiaohua@everest-semi.com>
 *													or
 *													<info@everest-semi.com>
 * Copyright:   (C) 2017 Everest Semiconductor Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ES7242_H
#define _ES7242_H

/* Codec TLV320AIC23 */
#define ES7242_MODECFG_REG00		0x00
#define ES7242_SDPFMT_REG01		0x01
#define ES7242_LRCDIV_REG02		0x02
#define ES7242_BCKDIV_REG03		0x03
#define ES7242_CLKDIV_REG04		0x04
#define ES7242_MUTECTL_REG05		0x05
#define ES7242_STATECTL_REG06			0x06
#define ES7242_ANACTL0_REG07		0x07
#define ES7242_ANACTL1_REG08		0x08
#define ES7242_ANACTL2_REG09		0x09
#define ES7242_ANACHARG_REG0A		0x0A
#define ES7242_INISTATE_REG0B		0x0B
#define ES7242_BIAS_REG0C		0x0C
#define ES7242_STMOSR_REG0D		0x0D
#define ES7242_CHIPID_REG0E		0x0E

#endif /* _ES7242_H_ */