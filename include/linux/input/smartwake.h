/*
 * include/linux/input/smartwake.h
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2015, Vineeth Raj <contact.twn@openmailbox.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _LINUX_SMARTWAKE_H
#define _LINUX_SMARTWAKE_H

extern int smartwake_switch;
extern bool smartwake_scr_suspended;
extern bool in_phone_call;
extern char wakeup_slide[32];
extern u32 support_gesture;

extern unsigned int smartwake_y_distance;
extern unsigned int smartwake_x_distance;
extern unsigned int min_delta;
extern unsigned int smartwake_vib;
extern int smartwake_switch;

extern int goodix_active(void);
extern int goodix_get_wakeup_gesture(char*  gesture);
extern int goodix_get_gesture_ctrl(char*  gesture_ctrl);
extern int goodix_gesture_ctrl(const char*  gesture_buf);

void smartwake_setdev(struct input_dev *);

#endif	/* _LINUX_SMARTWAKE_H */
