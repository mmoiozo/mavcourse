/*
 * Copyright (C) charles
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ext_colorfilter/ext_colorfilter.h"
 * @author charles
 * detect ext_colorfilter
 */

#ifndef EXT_COLORFILTER_H
#define EXT_COLORFILTER_H

#include <stdint.h>

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern int color_count;


extern float lcnt;  //leftcount
extern float clcnt; //leftcentrecount
extern float crcnt; //rightcentrecount
extern float rcnt; //rightcount

extern float avg_lcnt;  //leftcount
extern float avg_clcnt; //leftcentrecount
extern float avg_crcnt; //rightcentrecount
extern float avg_rcnt; //rightcount

extern void ext_colorfilter_init(void);

#endif

