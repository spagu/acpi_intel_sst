/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Intel SST Topology - Audio Pipeline Configuration
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/sound/pcm/sound.h>

#include "acpi_intel_sst.h"
#include "sst_topology.h"
#include "sst_ipc.h"

/*
 * dB-to-Q1.31 lookup table for widget volume control.
 * 169 entries: index 0 = -64.0dB, index 128 = 0dB (unity), index 168 = +20dB.
 * Each step = 0.5dB.  Volume parameter is in 0.5dB steps (-128..+40).
 * Formula: Q1.31 = round(0x7FFFFFFF * 10^(dB/20))
 * Values above 0dBFS are capped at 0x7FFFFFFF (unity).
 */
static const uint32_t sst_vol_db_to_q131[169] = {
	0x0014ACDB, 0x0015E67A, 0x001732AE, 0x00189292,
	0x001A074F, 0x001B9222, 0x001D345B, 0x001EEF5B,
	0x0020C49C, 0x0022B5AA, 0x0024C42C, 0x0026F1E1,
	0x002940A2, 0x002BB263, 0x002E4939, 0x00310756,
	0x0033EF0C, 0x003702D4, 0x003A454A, 0x003DB932,
	0x00416179, 0x0045413B, 0x00495BC1, 0x004DB486,
	0x00524F3B, 0x00572FC8, 0x005C5A4F, 0x0061D334,
	0x00679F1C, 0x006DC2F0, 0x007443E8, 0x007B2787,
	0x008273A6, 0x008A2E77, 0x00925E89, 0x009B0ACE,
	0x00A43AA2, 0x00ADF5D1, 0x00B8449C, 0x00C32FC3,
	0x00CEC08A, 0x00DB00C0, 0x00E7FACC, 0x00F5B9B0,
	0x01044915, 0x0113B557, 0x01240B8C, 0x01355991,
	0x0147AE14, 0x015B18A5, 0x016FA9BB, 0x018572CB,
	0x019C8651, 0x01B4F7E3, 0x01CEDC3D, 0x01EA4958,
	0x0207567A, 0x02261C4A, 0x0246B4E4, 0x02693BF0,
	0x028DCEBC, 0x02B48C50, 0x02DD958A, 0x03090D3F,
	0x0337184E, 0x0367DDCC, 0x039B8719, 0x03D2400C,
	0x040C3714, 0x04499D60, 0x048AA70B, 0x04CF8B44,
	0x05188480, 0x0565D0AB, 0x05B7B15B, 0x060E6C0B,
	0x066A4A53, 0x06CB9A26, 0x0732AE18, 0x079FDD9F,
	0x08138562, 0x088E0783, 0x090FCBF7, 0x099940DB,
	0x0A2ADAD1, 0x0AC51567, 0x0B68737A, 0x0C157FA9,
	0x0CCCCCCD, 0x0D8EF66D, 0x0E5CA14C, 0x0F367BEE,
	0x101D3F2D, 0x1111AEDB, 0x12149A60, 0x1326DD70,
	0x144960C5, 0x157D1AE2, 0x16C310E3, 0x181C5762,
	0x198A1357, 0x1B0D7B1B, 0x1CA7D768, 0x1E5A8471,
	0x2026F30F, 0x220EA9F4, 0x241346F6, 0x26368073,
	0x287A26C4, 0x2AE025C3, 0x2D6A866F, 0x301B70A8,
	0x32F52CFF, 0x35FA26A9, 0x392CED8E, 0x3C90386F,
	0x4026E73C, 0x43F4057E, 0x47FACCF0, 0x4C3EA838,
	0x50C335D3, 0x558C4B22, 0x5A9DF7AB, 0x5FFC8890,
	0x65AC8C2E, 0x6BB2D603, 0x721482BF, 0x78D6FC9E,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF,
};

/*
 * Convert widget dB (0.5dB steps: -128..+40) to Q1.31 linear gain.
 */
static uint32_t
sst_db_to_linear(int32_t volume_half_db)
{
	int idx;

	idx = volume_half_db + 128;	/* shift to 0-based: 0==-64dB, 128==0dB */
	if (idx < 0)
		idx = 0;
	if (idx > 168)
		idx = 168;
	return (sst_vol_db_to_q131[idx]);
}

/*
 * EQ preset biquad coefficients.
 *
 * Each preset is a single 2nd-order biquad (Q2.30 fixed-point) sent
 * to the catpt DSP via SET_BIQUAD IPC.  The DSP supports only one
 * biquad stage per stream, so each preset defines a complete filter.
 *
 * Coefficients: 2nd-order Butterworth HPF at 48kHz sample rate.
 * Formula:
 *   omega = 2*pi*fc/fs, alpha = sin(omega)/sqrt(2), a0 = 1+alpha
 *   b0 = (1+cos(omega))/2/a0,  b1 = -(1+cos(omega))/a0,  b2 = b0
 *   a1 = -2*cos(omega)/a0,     a2 = (1-alpha)/a0
 */
struct sst_eq_preset_entry {
	enum sst_eq_preset_id	id;
	const char		*name;
	int32_t			b0, b1, b2;	/* Numerator, Q2.30 */
	int32_t			a1, a2;		/* Denominator, Q2.30 */
	int			peak_gain_db;	/* max gain at any freq (dB) */
};

#define SST_EQ_NUM_PRESETS	3

static const struct sst_eq_preset_entry sst_eq_presets[SST_EQ_NUM_PRESETS] = {
	/* FLAT: bypass (b0=1.0, rest=0) */
	{ SST_EQ_PRESET_FLAT,
	  "flat",
	  1073741824, 0, 0, 0, 0, 0 },
	/* STOCK_SPEAKER: 2nd-order Butterworth HPF at 150Hz/48kHz */
	{ SST_EQ_PRESET_STOCK_SPEAKER,
	  "stock_speaker",
	  1058936991, -2117873982, 1058936991, -2117669842, 1044336297, 0 },
	/* MOD_SPEAKER: 2nd-order Butterworth HPF at 100Hz/48kHz */
	{ SST_EQ_PRESET_MOD_SPEAKER,
	  "mod_speaker",
	  1063849116, -2127698232, 1063849116, -2127607086, 1054047555, 0 },
};

/*
 * HPF cutoff lookup table: precomputed 2nd-order Butterworth HPF
 * biquad coefficients (Q2.30) at 48kHz sample rate.
 *
 * Snap user-requested cutoff to nearest table entry.
 * All HPFs have peak_gain_db = 0 (no boost).
 */
struct sst_hpf_coeff_entry {
	uint16_t  cutoff_hz;
	int32_t   b0, b1, b2, a1, a2;	/* Q2.30 */
	int       peak_gain_db;
};

#define SST_HPF_NUM_ENTRIES	13

static const struct sst_hpf_coeff_entry sst_hpf_table[SST_HPF_NUM_ENTRIES] = {
	/* 0 Hz: flat bypass (b0=1.0, rest=0) */
	{   0, 1073741824,           0,          0,          0,          0, 0 },
	/*  50 Hz */
	{  50, 1070557065, -2141114130, 1070557065, -2141073638, 1067412797, 0 },
	/*  60 Hz */
	{  60, 1069749283, -2139498567, 1069749283, -2139445397, 1065810023, 0 },
	/*  80 Hz */
	{  80, 1068102048, -2136204096, 1068102048, -2136117856, 1062549031, 0 },
	/* 100 Hz */
	{ 100, 1063849116, -2127698232, 1063849116, -2127607086, 1054047555, 0 },
	/* 120 Hz */
	{ 120, 1061401872, -2122803745, 1061401872, -2122664003, 1049203260, 0 },
	/* 150 Hz */
	{ 150, 1058936991, -2117873982, 1058936991, -2117669842, 1044336297, 0 },
	/* 200 Hz */
	{ 200, 1053339174, -2106678349, 1053339174, -2106318629, 1033298765, 0 },
	/* 250 Hz */
	{ 250, 1047779997, -2095559994, 1047779997, -2094984480, 1022400170, 0 },
	/* 300 Hz */
	{ 300, 1042260466, -2084520932, 1042260466, -2083668978, 1011642815, 0 },
	/* 350 Hz */
	{ 350, 1036781573, -2073563147, 1036781573, -2072374072, 1001028222, 0 },
	/* 400 Hz */
	{ 400, 1031344281, -2062688562, 1031344281, -2061101887,  990558028, 0 },
	/* 500 Hz */
	{ 500, 1020610704, -2041221409, 1020610704, -2038633771,  970082830, 0 },
};

/*
 * Snap requested frequency to nearest HPF table entry.
 * Returns table index.
 */
static int
sst_hpf_snap_freq(int freq)
{
	int i, best, best_diff, diff;

	if (freq <= 0)
		return (0);	/* bypass */

	best = 0;
	best_diff = abs(freq - sst_hpf_table[0].cutoff_hz);

	for (i = 1; i < SST_HPF_NUM_ENTRIES; i++) {
		diff = abs(freq - (int)sst_hpf_table[i].cutoff_hz);
		if (diff < best_diff) {
			best_diff = diff;
			best = i;
		}
	}

	return (best);
}

/*
 * PEQ (Parametric EQ) integer-only biquad coefficient computation.
 *
 * Peaking EQ transfer function (Audio EQ Cookbook):
 *   b0 = 1 + alpha*A    b1 = -2*cos(w0)    b2 = 1 - alpha*A
 *   a0 = 1 + alpha/A    a1 = -2*cos(w0)    a2 = 1 - alpha/A
 *   where A = 10^(gain_dB/40), w0 = 2*pi*fc/fs, alpha = sin(w0)/(2*Q)
 *
 * All arithmetic uses int32/int64 — no FPU.
 * Fixed-point format: Q2.30 (1.0 = 0x40000000).
 */

#define SST_Q30_ONE	0x40000000	/* 1.0 in Q2.30 */

/*
 * 256-entry Q1.30 sine lookup table (one quadrant: 0..pi/2).
 * sin_q30[i] = round(2^30 * sin(i * pi/2 / 256))
 */
static const int32_t sst_sin_q30[257] = {
	         0,   6588397,  13176560,  19764254,  26351244,  32937296,
	  39522175,  46105646,  52687476,  59267429,  65845272,  72420770,
	  78993690,  85563798,  92130862,  98694649, 105254925, 111811460,
	 118364021, 124912377, 131456298, 137995551, 144529906, 151059133,
	 157583002, 164101283, 170613746, 177120162, 183620302, 190113937,
	 196600839, 203080779, 209553530, 216018864, 222476554, 228926374,
	 235368098, 241801502, 248226360, 254642448, 261049543, 267447421,
	 273835860, 280214638, 286583533, 292942324, 299290791, 305628714,
	 311955874, 318272052, 324577029, 330870589, 337152516, 343422593,
	 349680606, 355926341, 362159584, 368380122, 374587744, 380782238,
	 386963394, 393131003, 399284855, 405424743, 411550461, 417661803,
	 423758565, 429840543, 435907534, 441959336, 447995749, 454016574,
	 460021612, 466010664, 471983535, 477940029, 483879950, 489803106,
	 495709304, 501598352, 507470060, 513324237, 519160696, 524979250,
	 530779712, 536561897, 542325622, 548070704, 553796963, 559504218,
	 565192291, 570861005, 576510183, 582139651, 587749235, 593338763,
	 598908064, 604456968, 609985306, 615492912, 620979618, 626445262,
	 631889679, 637312707, 642714187, 648093959, 653451866, 658787751,
	 664101460, 669392838, 674661733, 679907994, 685131472, 690332018,
	 695509486, 700663731, 705794607, 710901974, 715985688, 721045611,
	 726081604, 731093530, 736081253, 741044639, 745983553, 750897863,
	 755787439, 760652150, 765491868, 770306466, 775095818, 779859800,
	 784598289, 789311162, 793998300, 798659582, 803294891, 807904110,
	 812487125, 817043823, 821574092, 826077824, 830554909, 835005243,
	 839428722, 843825243, 848194707, 852537014, 856852069, 861139776,
	 865400041, 869632773, 873837882, 878015281, 882164885, 886286609,
	 890380371, 894446092, 898483693, 902493098, 906474232, 910427022,
	 914351397, 918247288, 922114628, 925953350, 929763393, 933544693,
	 937297192, 941020830, 944715551, 948381300, 952018024, 955625672,
	 959204194, 962753541, 966273667, 969764527, 973226076, 976658272,
	 980061075, 983434445, 986778343, 990092733, 993377580, 996632849,
	 999858510, 1003054531, 1006220882, 1009357536, 1012464466, 1015541646,
	1018589054, 1021606665, 1024594460, 1027552418, 1030480521, 1033378751,
	1036247094, 1039085534, 1041894060, 1044672659, 1047421322, 1050140039,
	1052828803, 1055487610, 1058116455, 1060715336, 1063284251, 1065823201,
	1068332187, 1070811212, 1073260282, 1075679400, 1078068574, 1080427812,
	1082757124, 1085056520, 1087326013, 1089565617, 1091775346, 1093955217,
	1096105248, 1098225457, 1100315864, 1102376492, 1104407363, 1106408501,
	1108379933, 1110321685, 1112233785, 1114116264, 1115969150, 1117792476,
	1119586275, 1121350581, 1123085429, 1124790857, 1126466901, 1128113602,
	1129730998, 1131319132, 1132878045, 1134407782, 1135908389, 1137379910,
	1138822394, 1140235889, 1141620445, 1142976112, 1144302943, 1145600991,
	1146870311, 1148110958, 1149322988, 1150506459,
	1151661430,
};

/*
 * Integer sine/cosine using quarter-wave LUT with linear interpolation.
 * Input: angle in Q2.30 radians (not normalized — raw omega value).
 * Output: Q2.30 result.
 *
 * We map the angle to a 0..2*pi range, then use quadrant folding
 * with the 256-entry quarter-wave table.
 *
 * For DSP biquad design at 48kHz, omega = 2*pi*fc/48000 is always
 * small (< pi), so we handle the full [0, 2*pi) range.
 */

/* pi in Q2.30: round(2^30 * pi) = 3373259426 (fits uint32_t) */
#define SST_PI_Q30	3373259426U
/* 2*pi in Q2.30 — wraps, so use uint32_t for mod arithmetic */
#define SST_2PI_Q30	(SST_PI_Q30 * 2U)
/* pi/2 in Q2.30 */
#define SST_HALFPI_Q30	(SST_PI_Q30 / 2U)

static int32_t
sst_sin_q30_interp(uint32_t angle_q30)
{
	uint32_t norm, idx_frac;
	int idx;
	int32_t s0, s1, frac, result;
	int negate;

	/* Reduce to [0, 2*pi) */
	if (angle_q30 >= SST_2PI_Q30)
		angle_q30 = angle_q30 % SST_2PI_Q30;

	/* Quadrant folding */
	negate = 0;
	if (angle_q30 >= SST_PI_Q30) {
		angle_q30 -= SST_PI_Q30;
		negate = 1;
	}
	if (angle_q30 > SST_HALFPI_Q30)
		angle_q30 = SST_PI_Q30 - angle_q30;

	/* Map [0, pi/2] to [0, 256] with fractional part */
	/* norm = angle * 256 / (pi/2) in Q2.30 */
	norm = (uint32_t)((uint64_t)angle_q30 * 256 / SST_HALFPI_Q30);

	/* Integer and fractional index */
	idx = (int)(norm >> 0);  /* This is 0..256 */
	if (idx > 256)
		idx = 256;

	/* For sub-entry interpolation, compute frac from higher precision */
	/* frac = (angle * 256 * 1024 / halfpi) & 0x3FF, scaled to Q30 */
	idx_frac = (uint32_t)((uint64_t)angle_q30 * 256 * 1024 /
	    SST_HALFPI_Q30);
	frac = (int32_t)(idx_frac & 0x3FF);  /* 10-bit fraction */

	s0 = sst_sin_q30[idx < 256 ? idx : 256];
	s1 = sst_sin_q30[idx < 256 ? idx + 1 : 256];

	/* Linear interpolation: s0 + (s1 - s0) * frac / 1024 */
	result = s0 + (int32_t)(((int64_t)(s1 - s0) * frac) >> 10);

	return (negate ? -result : result);
}

static int32_t
sst_cos_q30_interp(uint32_t angle_q30)
{
	return (sst_sin_q30_interp(angle_q30 + SST_HALFPI_Q30));
}

/*
 * 10^(gain_dB/40) lookup table for PEQ 'A' parameter.
 * 25 entries for gain = -12..+12 dB (index = gain + 12).
 * Q2.30 format: 1.0 = 0x40000000.
 *
 * A = 10^(g/40), used in peaking EQ:
 *   b0 = 1 + alpha*A, b2 = 1 - alpha*A
 *   a0 = 1 + alpha/A, a2 = 1 - alpha/A
 */
static const int32_t sst_peq_A_q30[25] = {
	 538145694,  /* -12 dB: A = 0.50119 */
	 570032831,  /* -11 dB: A = 0.53088 */
	 603809400,  /* -10 dB: A = 0.56234 */
	 639587356,  /*  -9 dB: A = 0.59566 */
	 677485290,  /*  -8 dB: A = 0.63096 */
	 717628817,  /*  -7 dB: A = 0.66834 */
	 760150998,  /*  -6 dB: A = 0.70795 */
	 805192776,  /*  -5 dB: A = 0.74989 */
	 852903448,  /*  -4 dB: A = 0.79433 */
	 903441154,  /*  -3 dB: A = 0.84140 */
	 956973408,  /*  -2 dB: A = 0.89125 */
	1013677647,  /*  -1 dB: A = 0.94406 */
	1073741824,  /*   0 dB: A = 1.00000 (0x40000000) */
	1137365027,  /*  +1 dB: A = 1.05925 */
	1204758142,  /*  +2 dB: A = 1.12202 */
	1276144550,  /*  +3 dB: A = 1.18850 */
	1351760868,  /*  +4 dB: A = 1.25893 */
	1431857735,  /*  +5 dB: A = 1.33352 */
	1516700640,  /*  +6 dB: A = 1.41254 */
	1606570803,  /*  +7 dB: A = 1.49624 */
	1701766107,  /*  +8 dB: A = 1.58489 */
	1802602089,  /*  +9 dB: A = 1.67880 */
	1909412977,  /* +10 dB: A = 1.77828 */
	2022552809,  /* +11 dB: A = 1.88365 */
	2142396597,  /* +12 dB: A = 1.99526 */
};

/*
 * Compute PEQ biquad coefficients (integer-only).
 *
 * Inputs:
 *   fc_hz:   center frequency in Hz (200..16000)
 *   gain_db: boost/cut in dB (-12..+12)
 *   q_x100:  Q factor × 100 (30..1000, e.g. 71 = 0.71)
 *   fs:      sample rate (typically 48000)
 *
 * Outputs:
 *   b0, b1, b2, a1, a2 in Q2.30
 *
 * Returns 0 on success.
 */
static int
sst_biquad_compute_peq(int fc_hz, int gain_db, int q_x100, int fs,
    int32_t *out_b0, int32_t *out_b1, int32_t *out_b2,
    int32_t *out_a1, int32_t *out_a2)
{
	uint32_t omega;		/* 2*pi*fc/fs in Q2.30 */
	int32_t sin_w, cos_w;
	int32_t A;		/* 10^(gain/40) in Q2.30 */
	int64_t alpha;		/* sin(w)/(2*Q) in Q2.30 */
	int64_t alphaA;		/* alpha * A in Q2.30 */
	int64_t alphaOverA;	/* alpha / A in Q2.30 */
	int64_t a0, b0, b1, b2, a1, a2;
	int a_idx;

	if (fc_hz < 200 || fc_hz > 16000)
		return (EINVAL);
	if (gain_db < -12 || gain_db > 12)
		return (EINVAL);
	if (q_x100 < 30 || q_x100 > 1000)
		return (EINVAL);

	/* omega = 2 * pi * fc / fs, in Q2.30 */
	omega = (uint32_t)((uint64_t)SST_2PI_Q30 * fc_hz / fs);

	/* sin(omega), cos(omega) via LUT */
	sin_w = sst_sin_q30_interp(omega);
	cos_w = sst_cos_q30_interp(omega);

	/* A = 10^(gain_db/40) from lookup table */
	a_idx = gain_db + 12;
	if (a_idx < 0) a_idx = 0;
	if (a_idx > 24) a_idx = 24;
	A = sst_peq_A_q30[a_idx];

	/* alpha = sin(w) / (2 * Q)
	 * Q = q_x100 / 100, so alpha = sin(w) * 100 / (2 * q_x100)
	 * = sin(w) * 50 / q_x100
	 * sin_w is Q2.30, result stays Q2.30
	 */
	alpha = (int64_t)sin_w * 50 / q_x100;

	/* alphaA = alpha * A (both Q2.30 → shift down 30) */
	alphaA = (alpha * (int64_t)A) >> 30;

	/* alphaOverA = alpha * Q30_ONE / A */
	alphaOverA = (alpha * (int64_t)SST_Q30_ONE) / A;

	/* Biquad coefficients (all in Q2.30 before normalization):
	 *   b0 = 1 + alpha*A
	 *   b1 = -2*cos(w)
	 *   b2 = 1 - alpha*A
	 *   a0 = 1 + alpha/A
	 *   a1 = -2*cos(w)
	 *   a2 = 1 - alpha/A
	 */
	b0 = SST_Q30_ONE + alphaA;
	b1 = -2 * (int64_t)cos_w;
	b2 = SST_Q30_ONE - alphaA;
	a0 = SST_Q30_ONE + alphaOverA;
	a1 = -2 * (int64_t)cos_w;
	a2 = SST_Q30_ONE - alphaOverA;

	/* Normalize: divide all by a0, keeping Q2.30 */
	if (a0 == 0)
		return (EINVAL);

	*out_b0 = (int32_t)((b0 << 30) / a0);
	*out_b1 = (int32_t)((b1 << 30) / a0);
	*out_b2 = (int32_t)((b2 << 30) / a0);
	*out_a1 = (int32_t)((a1 << 30) / a0);
	*out_a2 = (int32_t)((a2 << 30) / a0);

	return (0);
}

/*
 * Precomputed peak limiter threshold presets at 48kHz.
 * Q2.30 linear amplitude (range [-2.0, +2.0), 1.0 = 0x40000000).
 *
 * Formula: Q2.30 = round(2^30 * 10^(dB/20))
 *
 * Derived from Q1.31 volume table (Q2.30 = Q1.31 >> 1):
 *   -24dB: Q1.31[80]  = 0x08138562 → Q2.30 = 0x0409C2B1
 *   -18dB: Q1.31[92]  = 0x101D3F2D → Q2.30 = 0x080E9F97
 *   -12dB: Q1.31[104] = 0x2026F30F → Q2.30 = 0x10137988
 *    -9dB: Q1.31[110] = 0x2D6A866F → Q2.30 = 0x16B54338
 *    -6dB: Q1.31[116] = 0x4026E73C → Q2.30 = 0x2013739E
 *    -3dB: Q1.31[122] = 0x5A9DF7AB → Q2.30 = 0x2D4EFBD6
 *    -1dB: Q1.31[126] = 0x721482BF → Q2.30 = 0x390A4160
 *     0dB: 2^30 = 0x40000000
 *
 * Attack: fixed 1ms (1000us) for all presets (fast transient catch).
 * Release: scaled per preset (50ms at -24dB to 200ms at 0dB).
 */
struct sst_limiter_preset {
	uint32_t	threshold_db;		/* Threshold in dBFS (0=bypass) */
	int32_t		threshold_linear;	/* Q2.30 linear amplitude */
	uint32_t	attack_us;		/* Attack time in microseconds */
	uint32_t	release_us;		/* Release time in microseconds */
};

#define SST_LIMITER_NUM_PRESETS	9

static const struct sst_limiter_preset sst_limiter_presets[SST_LIMITER_NUM_PRESETS] = {
	{  0, 0x7FFFFFFF,    0,      0 },	/* 0: bypass (no limiting) */
	{ 24, 0x0409C2B1, 1000,  50000 },	/* 1: -24 dBFS */
	{ 18, 0x080E9F97, 1000,  75000 },	/* 2: -18 dBFS */
	{ 12, 0x10137988, 1000, 100000 },	/* 3: -12 dBFS */
	{  9, 0x16B54338, 1000, 115000 },	/* 4:  -9 dBFS */
	{  6, 0x2013739E, 1000, 135000 },	/* 5:  -6 dBFS (default) */
	{  3, 0x2D4EFBD6, 1000, 160000 },	/* 6:  -3 dBFS */
	{  1, 0x390A4160, 1000, 180000 },	/* 7:  -1 dBFS */
	{  0, 0x40000000, 1000, 200000 },	/* 8:   0 dBFS (full scale) */
};

/*
 * Default topology for Dell XPS 13 9343 / Broadwell-U
 *
 * Pipeline layout:
 *   Playback: Host -> PCM -> HPF -> Gain -> Limiter -> SSP0 DAI OUT -> Codec
 *   Capture:  Codec -> SSP1 DAI IN -> Gain -> PCM -> Host
 */

/*
 * Create default playback pipeline.
 *
 * The pipeline is built dynamically based on probed DSP capabilities:
 *   Always:      pcm0p (AIF_IN), PGA1.0 (Gain), ssp0-out (DAI_OUT)
 *   If biquad:   HPF1.0 inserted between pcm0p and PGA1.0
 *   If limiter:  LIMITER1.0 inserted between PGA1.0 and ssp0-out
 *
 * Resulting topologies:
 *   Full:    pcm0p -> HPF1.0 -> PGA1.0 -> LIMITER1.0 -> ssp0-out
 *   No HPF:  pcm0p -> PGA1.0 -> LIMITER1.0 -> ssp0-out
 *   No Lim:  pcm0p -> HPF1.0 -> PGA1.0 -> ssp0-out
 *   Minimal: pcm0p -> PGA1.0 -> ssp0-out
 */
static int
sst_topology_create_playback_pipe(struct sst_softc *sc)
{
	struct sst_topology *tplg = &sc->topology;
	struct sst_pipeline *pipe;
	struct sst_widget *w;
	struct sst_route *r;
	uint32_t idx;
	const char *prev;
	uint32_t route_count;
	bool has_hpf, has_lim;

	if (tplg->pipeline_count >= SST_TPLG_MAX_PIPELINES)
		return (ENOMEM);

	has_hpf = sc->fw.has_biquad;
	has_lim = sc->fw.has_limiter;

	idx = tplg->pipeline_count++;
	pipe = &tplg->pipelines[idx];

	memset(pipe, 0, sizeof(*pipe));
	strlcpy(pipe->name, "Playback", SST_TPLG_NAME_LEN);
	pipe->id = idx;
	pipe->type = SST_PIPE_PLAYBACK;
	pipe->state = SST_PIPE_STATE_CREATED;
	pipe->priority = 0;
	pipe->period_size = 1024;	/* 1024 frames */
	pipe->period_count = 4;
	pipe->ssp_port = 0;		/* SSP0 for playback */
	pipe->dma_channel = -1;
	pipe->core_id = 0;

	/* Create widgets for playback pipeline */

	/* Widget: Host PCM (input from host DMA) - always present */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "pcm0p", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_AIF_IN;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Widget: High-pass filter (speaker protection) - if supported */
	if (has_hpf && tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "HPF1.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_EFFECT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_HPF;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->eq_preset = SST_EQ_PRESET_STOCK_SPEAKER;
		pipe->widget_count++;
	}

	/* Widget: Gain/Volume control - always present */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "PGA1.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_PGA;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_GAIN;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->volume = 0;		/* 0dB default */
		w->mute = false;
		pipe->widget_count++;
	}

	/* Widget: Limiter (speaker protection) - if supported */
	if (has_lim && tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "LIMITER1.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_EFFECT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_LIMITER;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->limiter_threshold = 5;	/* Default -6 dBFS */
		pipe->widget_count++;
	}

	/* Widget: SSP0 DAI output (to codec) - always present */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "ssp0-out", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_DAI_OUT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/*
	 * Build routes dynamically using a prev_widget chain.
	 * Count needed routes: 2 (pcm0p->PGA, PGA->ssp0) + hpf + limiter
	 */
	route_count = 2 + (has_hpf ? 1 : 0) + (has_lim ? 1 : 0);
	if (tplg->route_count + route_count <= SST_TPLG_MAX_ROUTES) {
		prev = "pcm0p";

		if (has_hpf) {
			r = &tplg->routes[tplg->route_count++];
			strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
			strlcpy(r->sink, "HPF1.0", SST_TPLG_NAME_LEN);
			r->connected = true;
			prev = "HPF1.0";
		}

		/* prev -> PGA1.0 */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "PGA1.0", SST_TPLG_NAME_LEN);
		r->connected = true;
		prev = "PGA1.0";

		if (has_lim) {
			r = &tplg->routes[tplg->route_count++];
			strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
			strlcpy(r->sink, "LIMITER1.0", SST_TPLG_NAME_LEN);
			r->connected = true;
			prev = "LIMITER1.0";
		}

		/* prev -> ssp0-out */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "ssp0-out", SST_TPLG_NAME_LEN);
		r->connected = true;
	}

	device_printf(sc->dev,
	    "Topology: Created playback pipeline (id=%u) "
	    "widgets=%u [HPF=%s, Limiter=%s]\n",
	    pipe->id, pipe->widget_count,
	    has_hpf ? "yes" : "skipped",
	    has_lim ? "yes" : "skipped");

	return (0);
}

/*
 * Create default capture pipeline
 */
static int
sst_topology_create_capture_pipe(struct sst_softc *sc)
{
	struct sst_topology *tplg = &sc->topology;
	struct sst_pipeline *pipe;
	struct sst_widget *w;
	uint32_t idx;

	if (tplg->pipeline_count >= SST_TPLG_MAX_PIPELINES)
		return (ENOMEM);

	idx = tplg->pipeline_count++;
	pipe = &tplg->pipelines[idx];

	memset(pipe, 0, sizeof(*pipe));
	strlcpy(pipe->name, "Capture", SST_TPLG_NAME_LEN);
	pipe->id = idx;
	pipe->type = SST_PIPE_CAPTURE;
	pipe->state = SST_PIPE_STATE_CREATED;
	pipe->priority = 0;
	pipe->period_size = 1024;
	pipe->period_count = 4;
	pipe->ssp_port = 1;		/* SSP1 for capture */
	pipe->dma_channel = -1;
	pipe->core_id = 0;

	/* Create widgets for capture pipeline */

	/* Widget 1: SSP1 DAI input (from codec) */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "ssp1-in", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_DAI_IN;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Widget 2: Capture Gain control */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "PGA2.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_PGA;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_GAIN;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->volume = 0;
		w->mute = false;
		pipe->widget_count++;
	}

	/* Widget 3: Host PCM output (to host DMA) */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "pcm0c", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_AIF_OUT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Create routes for capture pipeline */
	if (tplg->route_count + 2 <= SST_TPLG_MAX_ROUTES) {
		struct sst_route *r;

		/* Route: ssp1-in -> PGA2.0 */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, "ssp1-in", SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "PGA2.0", SST_TPLG_NAME_LEN);
		r->connected = true;

		/* Route: PGA2.0 -> pcm0c */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, "PGA2.0", SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "pcm0c", SST_TPLG_NAME_LEN);
		r->connected = true;
	}

	device_printf(sc->dev, "Topology: Created capture pipeline (id=%u)\n",
		      pipe->id);

	return (0);
}

/*
 * Initialize topology subsystem
 */
int
sst_topology_init(struct sst_softc *sc)
{
	memset(&sc->topology, 0, sizeof(sc->topology));
	sc->topology.loaded = false;
	sc->topology.initialized = true;

	device_printf(sc->dev, "Topology subsystem initialized\n");

	return (0);
}

/*
 * Cleanup topology subsystem
 */
void
sst_topology_fini(struct sst_softc *sc)
{
	uint32_t i;

	if (!sc->topology.initialized)
		return;

	/* Destroy all pipelines */
	for (i = 0; i < sc->topology.pipeline_count; i++) {
		if (sc->topology.pipelines[i].state == SST_PIPE_STATE_RUNNING)
			sst_topology_stop_pipeline(sc, i);
		sst_topology_destroy_pipeline(sc, i);
	}

	memset(&sc->topology, 0, sizeof(sc->topology));
}

/*
 * Load default topology for Broadwell-U
 */
int
sst_topology_load_default(struct sst_softc *sc)
{
	int error;

	if (sc->topology.loaded) {
		device_printf(sc->dev, "Topology already loaded\n");
		return (EBUSY);
	}

	device_printf(sc->dev, "Loading default topology...\n");

	/* Create playback pipeline */
	error = sst_topology_create_playback_pipe(sc);
	if (error) {
		device_printf(sc->dev, "Failed to create playback pipeline\n");
		return (error);
	}

	/* Create capture pipeline */
	error = sst_topology_create_capture_pipe(sc);
	if (error) {
		device_printf(sc->dev, "Failed to create capture pipeline\n");
		return (error);
	}

	/* Resolve widget references in routes */
	for (uint32_t i = 0; i < sc->topology.route_count; i++) {
		struct sst_route *r = &sc->topology.routes[i];
		r->src_widget = sst_topology_find_widget(sc, r->source);
		r->dst_widget = sst_topology_find_widget(sc, r->sink);
	}

	sc->topology.loaded = true;

	device_printf(sc->dev, "Topology loaded: %u pipelines, %u widgets, %u routes\n",
		      sc->topology.pipeline_count,
		      sc->topology.widget_count,
		      sc->topology.route_count);

	return (0);
}

/*
 * Load topology from file (future extension)
 */
int
sst_topology_load_file(struct sst_softc *sc, const char *path)
{
	/* TODO: Parse ALSA topology format (.tplg) */
	device_printf(sc->dev, "Topology file loading not yet implemented: %s\n",
		      path);
	return (ENOTSUP);
}

/*
 * Create a pipeline on the DSP
 */
int
sst_topology_create_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state != SST_PIPE_STATE_CREATED &&
	    pipe->state != SST_PIPE_STATE_INVALID) {
		return (EBUSY);
	}

	/* Pipeline is ready - actual DSP setup happens on stream allocation */
	pipe->state = SST_PIPE_STATE_CREATED;

	device_printf(sc->dev, "Topology: Pipeline '%s' created\n", pipe->name);

	return (0);
}

/*
 * Destroy a pipeline
 */
int
sst_topology_destroy_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state == SST_PIPE_STATE_RUNNING)
		sst_topology_stop_pipeline(sc, pipe_id);

	/* Free stream if allocated */
	if (pipe->stream_allocated && sc->fw.state == SST_FW_STATE_RUNNING) {
		sst_ipc_free_stream(sc, pipe->stream_id);
		pipe->stream_allocated = false;
	}

	pipe->state = SST_PIPE_STATE_INVALID;

	return (0);
}

/*
 * Start a pipeline
 */
int
sst_topology_start_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;
	struct sst_alloc_stream_req req;
	int error;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state == SST_PIPE_STATE_RUNNING)
		return (0);

	if (pipe->state != SST_PIPE_STATE_CREATED &&
	    pipe->state != SST_PIPE_STATE_PAUSED) {
		return (EINVAL);
	}

	/* Stream allocation is now handled by sst_pcm via chan_trigger */
	(void)req;

	/* Resume stream */
	if (pipe->stream_allocated) {
		error = sst_ipc_stream_resume(sc, pipe->stream_id);
		if (error) {
			device_printf(sc->dev,
			    "Topology: Failed to resume stream for '%s'\n",
			    pipe->name);
			return (error);
		}
	}

	pipe->state = SST_PIPE_STATE_RUNNING;

	device_printf(sc->dev, "Topology: Pipeline '%s' started (stream=%u)\n",
		      pipe->name, pipe->stream_id);

	return (0);
}

/*
 * Stop a pipeline
 */
int
sst_topology_stop_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state != SST_PIPE_STATE_RUNNING)
		return (0);

	/* Pause stream */
	if (pipe->stream_allocated && sc->fw.state == SST_FW_STATE_RUNNING) {
		sst_ipc_stream_pause(sc, pipe->stream_id);
	}

	pipe->state = SST_PIPE_STATE_PAUSED;

	device_printf(sc->dev, "Topology: Pipeline '%s' stopped\n", pipe->name);

	return (0);
}

/*
 * Find widget by name
 */
struct sst_widget *
sst_topology_find_widget(struct sst_softc *sc, const char *name)
{
	uint32_t i;

	for (i = 0; i < sc->topology.widget_count; i++) {
		if (strcmp(sc->topology.widgets[i].name, name) == 0)
			return (&sc->topology.widgets[i]);
	}

	return (NULL);
}

/*
 * Set widget volume
 */
int
sst_topology_set_widget_volume(struct sst_softc *sc, struct sst_widget *w,
			       int32_t volume)
{
	struct sst_stream_params params;

	if (w == NULL)
		return (EINVAL);

	if (w->type != SST_WIDGET_PGA && w->type != SST_WIDGET_MIXER)
		return (EINVAL);

	/*
	 * Enforce headroom ceiling, accounting for EQ/PEQ peak gain.
	 *
	 * The base ceiling is -SST_HEADROOM_DB (-3dBFS in half-dB steps).
	 * If the pipeline's biquad boosts any frequency (HPF preset
	 * peak_gain_db or PEQ positive gain), reduce the volume ceiling
	 * by the same amount so EQ peak + volume never exceeds
	 * -SST_HEADROOM_DB dBFS.
	 */
	{
		int32_t ceiling;
		int peak = 0;

		ceiling = -SST_HEADROOM_HALF_DB;

		if (sc->pcm.biquad_mode == SST_BIQUAD_MODE_PEQ &&
		    sc->pcm.peq_gain > 0) {
			peak = sc->pcm.peq_gain;
		} else {
			struct sst_widget *hpf;
			hpf = sst_topology_find_widget(sc, "HPF1.0");
			if (hpf != NULL &&
			    hpf->pipeline_id == w->pipeline_id &&
			    hpf->eq_preset < SST_EQ_NUM_PRESETS)
				peak = sst_eq_presets[hpf->eq_preset].peak_gain_db;
		}
		if (peak > 0)
			ceiling -= peak * 2; /* dB to half-dB steps */
		if (volume > ceiling)
			volume = ceiling;
	}

	w->volume = volume;

	/* Update DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		uint32_t gain;

		memset(&params, 0, sizeof(params));
		params.stream_id = w->stream_id;
		gain = sst_db_to_linear(volume);
		params.volume_left = gain;
		params.volume_right = gain;
		params.mute = w->mute;

		return sst_ipc_stream_set_params(sc, &params);
	}

	return (0);
}

/*
 * Set widget EQ preset.
 * Looks up preset in the EQ table and sends biquad coefficients to DSP.
 */
int
sst_topology_set_widget_eq_preset(struct sst_softc *sc, struct sst_widget *w,
				  enum sst_eq_preset_id preset)
{
	const struct sst_eq_preset_entry *entry;

	if (w == NULL || w->module_type != SST_MOD_HPF)
		return (EINVAL);
	if (preset >= SST_EQ_NUM_PRESETS)
		preset = SST_EQ_PRESET_FLAT;

	entry = &sst_eq_presets[preset];
	w->eq_preset = preset;

	/*
	 * Gain budget enforcement: if the new preset boosts any
	 * frequency, pull the pipeline's PGA volume down so that
	 * EQ peak + volume stays within headroom.
	 */
	if (entry->peak_gain_db > 0) {
		struct sst_widget *pga;
		int32_t ceiling;

		ceiling = -(SST_HEADROOM_HALF_DB +
		    entry->peak_gain_db * 2);
		pga = sst_topology_find_widget(sc, "PGA1.0");
		if (pga != NULL && pga->pipeline_id == w->pipeline_id &&
		    pga->volume > ceiling)
			sst_topology_set_widget_volume(sc, pga, pga->volume);
	}

	/* Send biquad coefficients to DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		return sst_ipc_set_biquad(sc, w->stream_id,
		    entry->b0, entry->b1, entry->b2,
		    entry->a1, entry->a2);
	}

	return (0);
}

/*
 * Set widget limiter threshold.
 * Looks up threshold_idx in the preset table and sends parameters to DSP.
 */
int
sst_topology_set_widget_limiter(struct sst_softc *sc, struct sst_widget *w,
				uint32_t threshold_idx)
{
	const struct sst_limiter_preset *entry;

	if (w == NULL)
		return (EINVAL);

	if (w->module_type != SST_MOD_LIMITER)
		return (EINVAL);

	/* Clamp to valid preset range */
	if (threshold_idx >= SST_LIMITER_NUM_PRESETS)
		threshold_idx = SST_LIMITER_NUM_PRESETS - 1;
	entry = &sst_limiter_presets[threshold_idx];

	w->limiter_threshold = threshold_idx;

	/* Send limiter parameters to DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		return sst_ipc_set_limiter(sc, w->stream_id,
		    entry->threshold_linear, entry->attack_us,
		    entry->release_us);
	}

	return (0);
}

/*
 * Set widget limiter with explicit release override.
 * Uses threshold from preset table but substitutes custom release_us.
 */
int
sst_topology_set_widget_limiter_ex(struct sst_softc *sc, struct sst_widget *w,
				   uint32_t threshold_idx, uint32_t release_us)
{
	const struct sst_limiter_preset *entry;

	if (w == NULL || w->module_type != SST_MOD_LIMITER)
		return (EINVAL);

	if (threshold_idx >= SST_LIMITER_NUM_PRESETS)
		threshold_idx = SST_LIMITER_NUM_PRESETS - 1;
	entry = &sst_limiter_presets[threshold_idx];

	w->limiter_threshold = threshold_idx;

	/* Send with custom release time */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		return sst_ipc_set_limiter(sc, w->stream_id,
		    entry->threshold_linear, entry->attack_us,
		    release_us);
	}

	return (0);
}

/*
 * Set raw biquad coefficients on HPF widget (used by both HPF and PEQ).
 */
int
sst_topology_set_widget_biquad(struct sst_softc *sc, struct sst_widget *w,
			       int32_t b0, int32_t b1, int32_t b2,
			       int32_t a1, int32_t a2)
{
	if (w == NULL || w->module_type != SST_MOD_HPF)
		return (EINVAL);

	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		return sst_ipc_set_biquad(sc, w->stream_id,
		    b0, b1, b2, a1, a2);
	}

	return (0);
}

/*
 * Connect a route
 */
int
sst_topology_connect_route(struct sst_softc *sc, struct sst_route *r)
{
	if (r == NULL)
		return (EINVAL);

	r->connected = true;

	/* Actual DSP routing is done during pipeline setup */

	return (0);
}

/*
 * Disconnect a route
 */
int
sst_topology_disconnect_route(struct sst_softc *sc, struct sst_route *r)
{
	if (r == NULL)
		return (EINVAL);

	r->connected = false;

	return (0);
}

/*
 * Get pipeline for PCM stream
 */
struct sst_pipeline *
sst_topology_get_pipeline(struct sst_softc *sc, int dir, int stream_num)
{
	uint32_t i;
	enum sst_pipeline_type target_type;

	target_type = (dir == PCMDIR_PLAY) ? SST_PIPE_PLAYBACK : SST_PIPE_CAPTURE;

	for (i = 0; i < sc->topology.pipeline_count; i++) {
		if (sc->topology.pipelines[i].type == target_type)
			return (&sc->topology.pipelines[i]);
	}

	return (NULL);
}

/*
 * Helper: apply current biquad state (HPF or PEQ) to the HPF widget.
 * Called when any biquad-related sysctl changes.
 */
void
sst_topology_apply_biquad(struct sst_softc *sc)
{
	struct sst_widget *hpf_w;

	hpf_w = sst_topology_find_widget(sc, "HPF1.0");
	if (hpf_w == NULL)
		return;

	if (sc->pcm.biquad_mode == SST_BIQUAD_MODE_PEQ &&
	    sc->pcm.peq_freq > 0) {
		int32_t b0, b1, b2, a1, a2;
		int error;

		error = sst_biquad_compute_peq(sc->pcm.peq_freq,
		    sc->pcm.peq_gain, sc->pcm.peq_q, 48000,
		    &b0, &b1, &b2, &a1, &a2);
		if (error == 0)
			sst_topology_set_widget_biquad(sc, hpf_w,
			    b0, b1, b2, a1, a2);
	} else {
		/* HPF mode: use lookup table */
		int idx;
		const struct sst_hpf_coeff_entry *entry;

		idx = sst_hpf_snap_freq(sc->pcm.hpf_cutoff);
		entry = &sst_hpf_table[idx];
		sst_topology_set_widget_biquad(sc, hpf_w,
		    entry->b0, entry->b1, entry->b2,
		    entry->a1, entry->a2);
	}

	/*
	 * Update gain budget: PEQ boost requires lowering volume ceiling.
	 * Re-apply current volume so headroom logic kicks in.
	 * The ceiling calculation in set_widget_volume reads
	 * sc->pcm.biquad_mode and sc->pcm.peq_gain directly.
	 */
	{
		struct sst_widget *pga;

		pga = sst_topology_find_widget(sc, "PGA1.0");
		if (pga != NULL && hpf_w->pipeline_id == pga->pipeline_id)
			sst_topology_set_widget_volume(sc, pga, pga->volume);
	}
}

/*
 * Helper: apply current limiter state (with optional release override).
 */
void
sst_topology_apply_limiter(struct sst_softc *sc)
{
	struct sst_widget *lim_w;

	lim_w = sst_topology_find_widget(sc, "LIMITER1.0");
	if (lim_w == NULL)
		return;

	if (sc->pcm.limiter_release > 0) {
		sst_topology_set_widget_limiter_ex(sc, lim_w,
		    sc->pcm.limiter_threshold, sc->pcm.limiter_release);
	} else {
		sst_topology_set_widget_limiter(sc, lim_w,
		    sc->pcm.limiter_threshold);
	}
}

/*
 * Sysctl handler for EQ preset switching.
 * Reads/writes sc->pcm.eq_preset and applies to the HPF widget.
 * Writing eq_preset also sets hpf_cutoff to the preset's frequency
 * and switches to HPF mode.
 */
static int
sst_eq_preset_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int preset, error;

	preset = (int)sc->pcm.eq_preset;
	error = sysctl_handle_int(oidp, &preset, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (preset < 0 || preset >= SST_EQ_NUM_PRESETS)
		return (EINVAL);

	sc->pcm.eq_preset = (enum sst_eq_preset_id)preset;

	/* Map preset to HPF cutoff */
	switch (preset) {
	case SST_EQ_PRESET_FLAT:
		sc->pcm.hpf_cutoff = 0;
		break;
	case SST_EQ_PRESET_STOCK_SPEAKER:
		sc->pcm.hpf_cutoff = 150;
		break;
	case SST_EQ_PRESET_MOD_SPEAKER:
		sc->pcm.hpf_cutoff = 100;
		break;
	}

	/* Switch to HPF mode and apply */
	sc->pcm.biquad_mode = SST_BIQUAD_MODE_HPF;
	sc->pcm.peq_freq = 0;
	sst_topology_apply_biquad(sc);

	return (0);
}

/*
 * Sysctl handler for HPF cutoff frequency.
 * Overrides eq_preset. 0 = flat bypass, 50-500 = cutoff in Hz.
 */
static int
sst_hpf_cutoff_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int val, error;

	val = (int)sc->pcm.hpf_cutoff;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	/* Clamp to valid range */
	if (val < 0)
		val = 0;
	if (val > 0 && val < 50)
		val = 50;
	if (val > 500)
		val = 500;

	sc->pcm.hpf_cutoff = (uint16_t)val;

	/* Switch to HPF mode */
	sc->pcm.biquad_mode = SST_BIQUAD_MODE_HPF;
	sc->pcm.peq_freq = 0;
	sst_topology_apply_biquad(sc);

	return (0);
}

/*
 * Sysctl handler for limiter threshold preset index.
 * 0 = bypass, 1-8 = threshold presets.
 */
static int
sst_limiter_threshold_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int val, error;

	val = (int)sc->pcm.limiter_threshold;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (val < 0 || val >= SST_LIMITER_NUM_PRESETS)
		return (EINVAL);

	sc->pcm.limiter_threshold = (uint32_t)val;
	sst_topology_apply_limiter(sc);

	return (0);
}

/*
 * Sysctl handler for limiter release time override.
 * 10000-500000 µs. Overrides the preset's default release.
 * Set to 0 to revert to preset default.
 */
static int
sst_limiter_release_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int val, error;

	val = (int)sc->pcm.limiter_release;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (val != 0 && (val < 10000 || val > 500000))
		return (EINVAL);

	sc->pcm.limiter_release = (uint32_t)val;
	sst_topology_apply_limiter(sc);

	return (0);
}

/*
 * Sysctl handler for PEQ center frequency.
 * 0 = off (reverts to HPF mode), 200-16000 Hz = PEQ active.
 */
static int
sst_peq_freq_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int val, error;

	val = (int)sc->pcm.peq_freq;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (val != 0 && (val < 200 || val > 16000))
		return (EINVAL);

	sc->pcm.peq_freq = (uint16_t)val;

	if (val > 0) {
		sc->pcm.biquad_mode = SST_BIQUAD_MODE_PEQ;
		/* Initialize defaults if not already set */
		if (sc->pcm.peq_q == 0)
			sc->pcm.peq_q = 71;	/* Q = 0.71 */
	} else {
		sc->pcm.biquad_mode = SST_BIQUAD_MODE_HPF;
	}

	sst_topology_apply_biquad(sc);

	return (0);
}

/*
 * Sysctl handler for PEQ gain in dB.
 * -12 to +12 dB.
 */
static int
sst_peq_gain_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int val, error;

	val = sc->pcm.peq_gain;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (val < -12 || val > 12)
		return (EINVAL);

	sc->pcm.peq_gain = val;

	/* Only recompute if PEQ is active */
	if (sc->pcm.biquad_mode == SST_BIQUAD_MODE_PEQ &&
	    sc->pcm.peq_freq > 0)
		sst_topology_apply_biquad(sc);

	return (0);
}

/*
 * Sysctl handler for PEQ Q factor (× 100).
 * 30-1000 (e.g. 71 = Q 0.71, 141 = Q 1.41).
 */
static int
sst_peq_q_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int val, error;

	val = sc->pcm.peq_q;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (val < 30 || val > 1000)
		return (EINVAL);

	sc->pcm.peq_q = val;

	/* Only recompute if PEQ is active */
	if (sc->pcm.biquad_mode == SST_BIQUAD_MODE_PEQ &&
	    sc->pcm.peq_freq > 0)
		sst_topology_apply_biquad(sc);

	return (0);
}

/*
 * Register all DSP parameter sysctls under device tree.
 * Creates dev.acpi_intel_sst.N.{eq_preset,hpf_cutoff,...} (RW).
 */
int
sst_topology_sysctl_init(struct sst_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "eq_preset", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_eq_preset_sysctl, "I",
	    "EQ preset (0=flat, 1=stock_speaker, 2=mod_speaker)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "hpf_cutoff", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_hpf_cutoff_sysctl, "I",
	    "HPF cutoff frequency in Hz (0=flat, 50-500)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "limiter_threshold", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_limiter_threshold_sysctl, "I",
	    "Limiter preset index (0=bypass, 1-8)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "limiter_release", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_limiter_release_sysctl, "I",
	    "Limiter release time in us (0=preset default, 10000-500000)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "peq_freq", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_peq_freq_sysctl, "I",
	    "PEQ center frequency in Hz (0=off, 200-16000)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "peq_gain", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_peq_gain_sysctl, "I",
	    "PEQ boost/cut in dB (-12 to +12)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "peq_q", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_peq_q_sysctl, "I",
	    "PEQ Q factor x100 (30-1000, e.g. 71=0.71)");

	return (0);
}
