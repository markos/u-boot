/*
 * Copyright 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 2 as published by the Free Software Foundation.
 */

#ifndef COMMON_TIMING_PARAMS_H
#define COMMON_TIMING_PARAMS_H

typedef struct {
	/* parameters to constrict */

	unsigned int tckmin_x_ps;
	unsigned int tckmax_ps;
	unsigned int tckmax_max_ps;
	unsigned int trcd_ps;
	unsigned int trp_ps;
	unsigned int tras_ps;

	unsigned int twr_ps;	/* maximum = 63750 ps */
	unsigned int twtr_ps;	/* maximum = 63750 ps */
	unsigned int trfc_ps;	/* maximum = 255 ns + 256 ns + .75 ns
					   = 511750 ps */

	unsigned int trrd_ps;	/* maximum = 63750 ps */
	unsigned int trc_ps;	/* maximum = 254 ns + .75 ns = 254750 ps */

	unsigned int refresh_rate_ps;

	unsigned int tis_ps;	/* byte 32, spd->ca_setup */
	unsigned int tih_ps;	/* byte 33, spd->ca_hold */
	unsigned int tds_ps;	/* byte 34, spd->data_setup */
	unsigned int tdh_ps;	/* byte 35, spd->data_hold */
	unsigned int trtp_ps;	/* byte 38, spd->trtp */
	unsigned int tdqsq_max_ps;	/* byte 44, spd->tdqsq */
	unsigned int tqhs_ps;	/* byte 45, spd->tqhs */

	unsigned int ndimms_present;
	unsigned int lowest_common_SPD_caslat;
	unsigned int highest_common_derated_caslat;
	unsigned int additive_latency;
	unsigned int all_dimms_burst_lengths_bitmask;
	unsigned int all_dimms_registered;
	unsigned int all_dimms_unbuffered;
	unsigned int all_dimms_ecc_capable;

	unsigned long long total_mem;
	unsigned long long base_address;

	/* DDR3 RDIMM */
	unsigned char rcw[16];	/* Register Control Word 0-15 */
} common_timing_params_t;

#endif
