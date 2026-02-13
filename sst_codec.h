/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * RT286 (ALC3263) Codec Driver over I2C
 * Target: Intel Broadwell-U with Realtek RT286 on I2C0
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _SST_CODEC_H_
#define _SST_CODEC_H_

#include <sys/types.h>
#include <machine/bus.h>

/*
 * RT286 HDA Verb Encoding Macros
 *
 * HDA verbs are encoded as: (NID << 24) | (verb << 8) | param
 * For "set" verbs the format varies; we use explicit register addresses.
 */
#define RT286_VERB_CMD(nid, verb, param) \
	(((uint32_t)(nid) << 24) | ((uint32_t)(verb) << 8) | (uint32_t)(param))

/*
 * HDA Node IDs (NIDs) for RT286
 */
#define RT286_NID_ROOT		0x00	/* Root node */
#define RT286_NID_AFG		0x01	/* Audio Function Group */
#define RT286_NID_DAC0		0x02	/* DAC0 - front/speaker */
#define RT286_NID_DAC1		0x03	/* DAC1 - headphone */
#define RT286_NID_ADC0		0x08	/* ADC0 */
#define RT286_NID_ADC1		0x09	/* ADC1 */
#define RT286_NID_MIX		0x0C	/* Output mixer (front) */
#define RT286_NID_SPK		0x14	/* Speaker pin */
#define RT286_NID_MIC		0x18	/* Mic pin */
#define RT286_NID_HP		0x21	/* Headphone pin */

/*
 * HDA Verb Register Addresses (pre-encoded as 32-bit I2C register values)
 *
 * rl6347a I2C format: (NID << 24) | verb_data[23:0]
 * For GET verbs, bit 19 is set by the read function.
 */

/* Power State (verb 0x705 set, 0xF05 get) */
#define RT286_SET_POWER(nid)	(((uint32_t)(nid) << 24) | 0x70500)
#define RT286_GET_POWER(nid)	(((uint32_t)(nid) << 24) | 0xF0500)

/* Pin Widget Control (verb 0x707 set, 0xF07 get) */
#define RT286_SET_PIN_CTRL(nid)	(((uint32_t)(nid) << 24) | 0x70700)
#define RT286_GET_PIN_CTRL(nid)	(((uint32_t)(nid) << 24) | 0xF0700)

/* EAPD/BTL Enable (verb 0x70C set, 0xF0C get) */
#define RT286_SET_EAPD(nid)	(((uint32_t)(nid) << 24) | 0x70C00)

/* Amp Gain/Mute - Output (verb 0x3 set, left/right encoded in data) */
/* Left:  (NID << 24) | 0xA0 << 8 | gain  (bit 15=out, bit 13=left, bit 12=set) */
/* Right: (NID << 24) | 0x90 << 8 | gain  (bit 15=out, bit 14=right, bit 12=set) */
#define RT286_SET_AMP_OUT_L(nid)  (((uint32_t)(nid) << 24) | 0x3A000)
#define RT286_SET_AMP_OUT_R(nid)  (((uint32_t)(nid) << 24) | 0x39000)
#define RT286_SET_AMP_OUT_LR(nid) (((uint32_t)(nid) << 24) | 0x3B000)

/* Connection Select (verb 0x701) */
#define RT286_SET_CONNECT(nid)	(((uint32_t)(nid) << 24) | 0x70100)

/* Stream Format (verb 0x2 set, 0xA get) */
#define RT286_SET_FORMAT(nid)	(((uint32_t)(nid) << 24) | 0x20000)

/* Get Parameter (verb 0xF00) */
#define RT286_GET_PARAM(nid, param)  (((uint32_t)(nid) << 24) | 0xF0000 | (param))

/* GPIO (on AFG NID 0x01) */
#define RT286_SET_GPIO_MASK	(((uint32_t)0x01 << 24) | 0x71600)
#define RT286_SET_GPIO_DIR	(((uint32_t)0x01 << 24) | 0x71700)
#define RT286_SET_GPIO_DATA	(((uint32_t)0x01 << 24) | 0x71500)

/* Mixer unmute (verb 0x370 on NID 0x0C, index in low bits) */
#define RT286_SET_MIX_DAC	(((uint32_t)0x0C << 24) | 0x37000)

/*
 * Index Register Access (via Vendor-defined verb)
 *
 * COEF_INDEX = verb 0x500 on NID 0x20
 * PROC_COEF  = verb 0x400/0xC00 (set/get) on NID 0x20
 */
#define RT286_COEF_INDEX	(((uint32_t)0x20 << 24) | 0x50000)
#define RT286_PROC_COEF_SET	(((uint32_t)0x20 << 24) | 0x40000)
#define RT286_PROC_COEF_GET	(((uint32_t)0x20 << 24) | 0xC0000)

/*
 * Index Register Addresses
 * Written to COEF_INDEX before reading/writing PROC_COEF
 */
#define RT286_IDX_POWER_CTRL	0x01	/* Power control */
#define RT286_IDX_POWER_CTRL2	0x02	/* Power control 2 */
#define RT286_IDX_POWER_CTRL3	0x03	/* Power control 3 */
#define RT286_IDX_I2S_CTRL1	0x09	/* I2S control */
#define RT286_IDX_I2S_CTRL2	0x0A	/* I2S control 2 */
#define RT286_IDX_DC_GAIN	0x0D	/* DC gain calibration */
#define RT286_IDX_A_BIAS_CTRL1	0x10	/* Analog bias 1 */
#define RT286_IDX_A_BIAS_CTRL2	0x11	/* Analog bias 2 */
#define RT286_IDX_A_BIAS_CTRL3	0x12	/* Analog bias 3 */
#define RT286_IDX_A_BIAS_CTRL4	0x13	/* Analog bias 4 */
#define RT286_IDX_PLL_CTRL	0x16	/* PLL control */
#define RT286_IDX_GPIO_CTRL	0x29	/* GPIO control */
#define RT286_IDX_DEPOP_CTRL1	0x67	/* Depop control 1 */
#define RT286_IDX_DEPOP_CTRL2	0x68	/* Depop control 2 */
#define RT286_IDX_DEPOP_CTRL3	0x69	/* Depop control 3 */
#define RT286_IDX_CBJ		0x4F	/* Combo jack config */
#define RT286_IDX_MISC_CTRL	0x20	/* Misc control */

/*
 * Power state values (HDA spec)
 */
#define RT286_PWR_D0		0x00	/* Full on */
#define RT286_PWR_D1		0x01	/* Low power */
#define RT286_PWR_D2		0x02	/* Prepare for D3 */
#define RT286_PWR_D3		0x03	/* Off */

/*
 * Pin control bits
 */
#define RT286_PIN_OUT		0x40	/* Output enable */
#define RT286_PIN_HP		0x80	/* Headphone amp enable */
#define RT286_PIN_IN		0x20	/* Input enable */
#define RT286_PIN_VREF_80	0x04	/* VRef 80% */

/*
 * DAC Stream Format: 48kHz, 16-bit, 2 channels
 * Format register value: 0x0011
 *   bits[15:14] = 00 (PCM)
 *   bits[13:11] = 000 (48kHz base)
 *   bits[10:8]  = 000 (multiply x1)
 *   bits[7]     = 0 (divisor 1)
 *   bits[6:4]   = 001 (16-bit)
 *   bits[3:0]   = 0001 (2 channels)
 */
#define RT286_FMT_48K_16B_2CH	0x0011

/*
 * Dell GPIO6 for speaker amplifier
 */
#define RT286_DELL_GPIO6	0x40

/*
 * RT288 (ALC288) variant - same register layout as RT286
 */
#define RT288_VENDOR_ID		0x10EC0288

/*
 * Codec State Structure
 */
struct sst_codec {
	bus_space_tag_t		mem_tag;	/* Memory tag for I2C */
	bus_space_handle_t	i2c_handle;	/* I2C MMIO handle */
	bool			i2c_mapped;	/* I2C region mapped */
	bool			initialized;	/* Codec init complete */
	bool			speaker_active;	/* Speaker output enabled */
	bool			hp_active;	/* Headphone output enabled */
	uint32_t		vendor_id;	/* Detected vendor ID */
};

/* Forward declaration */
struct sst_softc;

/*
 * Codec API
 */
int	sst_codec_init(struct sst_softc *sc);
void	sst_codec_fini(struct sst_softc *sc);
int	sst_codec_enable_speaker(struct sst_softc *sc);
int	sst_codec_enable_headphone(struct sst_softc *sc);

#endif /* _SST_CODEC_H_ */
