#ifndef _LINUX_DOUBLETAP2WAKE_H
#define _LINUX_DOUBLETAP2WAKE_H

extern int dt2w_switch;
extern bool dt2w_scr_suspended;
extern bool in_phone_call, phone_call_override;
extern int vib_strength;
extern unsigned int left, right, co_up, co_down, dt2w_radius, dt2w_time, Dt2w_regions;
extern unsigned int vibration_strength_on_pocket_override, pocket_override_timeout, dt2w_override_taps;
extern bool revert_area;

void doubletap2wake_setdev(struct input_dev *);

#endif	/* _LINUX_DOUBLETAP2WAKE_H */
