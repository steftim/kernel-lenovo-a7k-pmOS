#ifndef _LINUX_KLAPSE_EXT_REF_H
#define _LINUX_KLAPSE_EXT_REF_H

#include "klapse.h"

// Klapse external references for outside use (Please know what you're doing with it)
extern int daytime_r, daytime_g, daytime_b, target_r, target_g, target_b;
extern int livedisplay_start_hour, livedisplay_stop_hour, force_livedisplay;
extern int brightness_lvl_auto_hour_start, brightness_lvl_auto_hour_end;
extern unsigned int livedisplay_aggression, brightness_lvl, klapse_brightness_threshold;
extern bool brightness_lvl_auto_enable;
extern void calc_active_minutes(void);
extern bool target_achieved;
extern struct rtc_time tm;
extern unsigned int b_cache;
extern void force_livedisplay_set_rgb_brightness(int r,int g,int b);
extern void daytime_rgb_updated(int r,int b, int g);
extern void set_force_livedisplay(int val);

#endif	/* _LINUX_KLAPSE_EXT_REF_H */
