#ifndef _LINUX_POCKET_MOD_H
#define _LINUX_POCKET_MOD_H

extern int is_screen_on;
extern char alsps_dev;
extern bool in_phone_call;
extern bool pocket_mod_switch;

int epl2182_pocket_detection_check(void);

int device_is_pocketed(void);

#endif //_LINUX_POCKET_MOD_H
