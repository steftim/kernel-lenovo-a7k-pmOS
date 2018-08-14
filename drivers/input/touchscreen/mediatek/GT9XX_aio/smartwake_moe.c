/*
 * drivers/input/touchscreen/smartwake.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2015, Vineeth Raj <contact.twn@openmailbox.org>
 * Copyright (c) 2016, Dominik Baronelli <dominik.baronelli@gmail.com>
 * Copyright (c) 2017, Godavarti Sandeep <sandeepg2404@gmail.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/input/smartwake.h>
#include "touchscreen.h"

#ifdef CONFIG_POCKETMOD
#include <linux/pocket_mod.h>
#endif
 
#define WAKE_HOOKS_DEFINED

#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
#include <linux/lcd_notify.h>
#else
#include <linux/earlysuspend.h>
#endif
#endif


/* Version, author, desc, etc */
#define DRIVER_AUTHOR "tanish2k09 (based on initiation by DerTeufel1980 <dominik.baronelli@gmail.com>"
#define DRIVER_DESCRIPTION "generic smartwake driver"
#define DRIVER_VERSION "1.1"
#define LOGTAG "[GT9XX]: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

/* Tuneables */
#define SMARTWAKE_DEBUG                 0
#define SMARTWAKE_DEFAULT               0
#define SMARTWAKE_KEY_DUR               60

#define SMARTWAKE_Y_DISTANCE            400
#define SMARTWAKE_X_DISTANCE            400
#define MIN_DELTA                       5
#define VIB_STRENGTH					50

/* directions */
#define RIGHT           1
#define RIGHT_UP        2
#define UP              4
#define LEFT_UP         8
#define LEFT            16
#define LEFT_DOWN       32
#define DOWN            64
#define RIGHT_DOWN      128

/* max gestures which can be stored */
#define MAX_GESTURES    15
#define FIFTEEN_ZEROES 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

//My code :
extern void set_vibrate(int value);

unsigned int smartwake_y_distance = SMARTWAKE_Y_DISTANCE;
unsigned int smartwake_x_distance = SMARTWAKE_X_DISTANCE;
unsigned int min_delta = MIN_DELTA;
unsigned int smartwake_vib = 50;
///////////


static int passed_gestures[MAX_GESTURES] = { FIFTEEN_ZEROES };
static int x_i[MAX_GESTURES] = { FIFTEEN_ZEROES };
static int y_i[MAX_GESTURES] = { FIFTEEN_ZEROES };
enum support_gesture_e {
	TW_SUPPORT_NONE_SLIDE_WAKEUP = 0x0,
	TW_SUPPORT_E_SLIDE_WAKEUP = 0x10,
	TW_SUPPORT_O_SLIDE_WAKEUP = 0x20,
	TW_SUPPORT_M_SLIDE_WAKEUP = 0x100,

	TW_SUPPORT_GESTURE_IN_ALL = (
			TW_SUPPORT_E_SLIDE_WAKEUP |
			TW_SUPPORT_O_SLIDE_WAKEUP |
			TW_SUPPORT_M_SLIDE_WAKEUP)
};

u32 support_gesture = TW_SUPPORT_GESTURE_IN_ALL;
char wakeup_slide[32];

static int smartwake_gesture_keys[] = {
	KEY_GESTURE_SLIDE_O,
	KEY_GESTURE_SLIDE_M,
	KEY_GESTURE_SLIDE_E,
	KEY_POWER
};
#define TPD_GESTRUE_KEY_CNT (sizeof( smartwake_gesture_keys )/sizeof( smartwake_gesture_keys[0] ))

/* Resources */
int smartwake_switch = SMARTWAKE_DEFAULT;
static int touch_x = 0, touch_y = 0; 
static int start_x = 0, start_y = 0, old_direction = 0, end_x = 0, end_y = 0, finger_id = -1;
static int prev_x = 0, prev_y = 0;
static int direction[3] = { 0,0,0 }; 
static int key_value = 0;
static bool touch_x_called = false, touch_y_called = false, direction_changed = false, onlyGesture = false;
static bool exec_count = true;
static bool scr_on_touch = false;
bool smartwake_scr_suspended = false;
#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
static struct notifier_block smartwake_lcd_notif;
#endif
#endif
static struct input_dev * smartwake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *smartwake_input_wq;
static struct work_struct smartwake_input_work;

/* Read cmdline for smartwake */
static int __init read_smartwake_cmdline(char *smartwake)
{
        if (strcmp(smartwake, "1") == 0) {
                pr_info("[cmdline_smartwake]: Smartwake enabled. | smartwake='%s'\n", smartwake);
                smartwake_switch = 1;
        } else if (strcmp(smartwake, "0") == 0) {
                pr_info("[cmdline_smartwake]: Smartwake disabled. | smartwake='%s'\n", smartwake);
                smartwake_switch = 0;
        } else {
                pr_info("[cmdline_smartwake]: No valid input found. Going with default: | smartwake='%u'\n", smartwake_switch);
        }
        return 1;
}
__setup("smartwake=", read_smartwake_cmdline);

/* PowerKey work func */
static void smartwake_presskey(struct work_struct * smartwake_presskey_work) {
        if (!mutex_trylock(&pwrkeyworklock)) {
                key_value = 0;
                return;
        }
    #if SMARTWAKE_DEBUG
    pr_info(LOGTAG"smartwake_presspw: key_value(%4d)\n", key_value);
    #endif
	input_report_key(smartwake_pwrdev, key_value, 1);
	input_sync(smartwake_pwrdev);
	msleep(SMARTWAKE_KEY_DUR);
	input_report_key(smartwake_pwrdev, key_value, 0);
	input_sync(smartwake_pwrdev);
	msleep(SMARTWAKE_KEY_DUR);
	key_value = 0;
	mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(smartwake_presskey_work, smartwake_presskey);

/* PowerKey trigger */
static void smartwake_keytrigger(void) {
        set_vibrate(smartwake_vib);
        schedule_work(&smartwake_presskey_work);
        return;
}

/* reset on finger release */
static void smartwake_reset(void) {
        int i;
        //check_gesture();
        finger_id = -1;
        exec_count = true;
        scr_on_touch = false;
        start_x = 0;
        start_y = 0;
        prev_x = 0;
        prev_y = 0;
        end_x = 0;
        end_y = 0;
        touch_x=0;
        touch_y=0;
        touch_x_called=0;
        touch_y_called=0;
        old_direction = 0;
        for (i = 0; i < 3; i++) {
                direction[i] = 0;
        }
        direction_changed = false;
        onlyGesture = false;
        for (i = 0; i < MAX_GESTURES; i++) {
                passed_gestures[i] = 0;
                x_i[i] = 0;
                y_i[i] = 0;
        }
}

static bool gesture_is_o(int count)
{
        int i = 0;
        int clockwise = 0;
        int sum = 0;
        int min_y = 0;
        for (i = 0; i + 1 < count; i++) {
                if ((2 * passed_gestures[i] == passed_gestures[i + 1] || (passed_gestures[i] == 128 && passed_gestures[i + 1] == 1)) &&
                        clockwise < 1) 
                {
                        clockwise = -1;
                        sum += passed_gestures[i];
                        if (y_i[i] != 0 && y_i[i] > min_y) // y=0 at top of the screen
                                min_y = y_i[i];

                }
                else if ((passed_gestures[i] == 2 * passed_gestures[i + 1] || (passed_gestures[i + 1] == 128 && passed_gestures[i] == 1)) &&
                        clockwise > -1) 
                {
                        clockwise = 1;
                        sum += passed_gestures[i];
                        if (y_i[i] != 0 && y_i[i] > min_y) // y=0 at top of the screen
                                min_y = y_i[i];
                }
                if (sum == 255 && end_y < min_y + 50) { // if last touch is lower than lowest touch of the circle, this was probably meant to be an 'e'
                        return true;
                } else if (sum > 255) {
                        return false;
                }
        }
        return false;
}

static bool gesture_is_m(int count)
{
        int i = 0;
        int sum = 0;
        bool below_part = false;
        for (i = 0; i < count - 1; i++) {
                if (!below_part && 
                        (passed_gestures[i] == 2 * passed_gestures[i + 1] || (passed_gestures[i] == 1 && passed_gestures[i + 1] == 128) ||
                        (passed_gestures[i] == DOWN && passed_gestures[i + 1] == UP)))
                {
                        sum += passed_gestures[i];
                        if (i + 2 == count) {
                                sum += passed_gestures[i + 1];
                                i += 1;
                        }
                }
                else if (!below_part && passed_gestures[i] == DOWN && passed_gestures[i + 1] == RIGHT_DOWN &&
                        (sum + DOWN == 199 || sum + DOWN == 195))
                {
                        sum += passed_gestures[i];
                        below_part = true;
                        
                }
                else if(below_part && 
                        ((passed_gestures[i] == RIGHT_DOWN && passed_gestures[i + 1] == RIGHT) || 
                        (passed_gestures[i] == RIGHT && passed_gestures[i + 1] == RIGHT_UP)))
                {
                        sum += passed_gestures[i];
                        if (sum == 328 && passed_gestures[i] == RIGHT) {
                                below_part = false;
                        }
                }               
                
                if ((sum == 398 && i == 9) || (sum == 394 && i == 8) || (sum == 334 && i == 8) || (sum == 330 && i == 7) ||
                        (sum == 529 && i == 12) || (sum == 525 && i == 11) || (sum == 465 && i == 11) || (sum == 461 && i == 10))
                {
                        return true;
                } 
        }
        return false;
}

static void check_gesture(void)
{
        int i = 0; 
        int gesture_sum = 0;
        int gesture_count = 0;
        key_value = 0;
        for (i = 0; i < MAX_GESTURES; i++) {
#if SMARTWAKE_DEBUG
        switch(passed_gestures[i]) {
                case 1: 
                        pr_info(LOGTAG"passed_gestures[%d]: RIGHT\n", i);
                        break;
                case 2: 
                        pr_info(LOGTAG"passed_gestures[%d]: RIGHT_UP\n", i);
                        break;
                case 4: 
                        pr_info(LOGTAG"passed_gestures[%d]: UP\n", i);
                        break;
                case 8: 
                        pr_info(LOGTAG"passed_gestures[%d]: LEFT_UP\n", i);
                        break;
                case 16:
                        pr_info(LOGTAG"passed_gestures[%d]: LEFT\n", i);
                        break;
                case 32: 
                        pr_info(LOGTAG"passed_gestures[%d]: LEFT_DOWN\n", i);
                        break;
                case 64: 
                        pr_info(LOGTAG"passed_gestures[%d]: DOWN\n", i);
                        break;
                case 128: 
                        pr_info(LOGTAG"passed_gestures[%d]: RIGHT_DOWN\n", i);
                        break;
        }
#endif
                if (passed_gestures[i] != 0) {
                        gesture_sum += passed_gestures[i];
                        gesture_count = i + 1;
                } else {
                        break;
                }
        }
#if SMARTWAKE_DEBUG
        pr_info(LOGTAG"gesture_count= (%d), gesture_sum = (%d), abs(end_x - start_x) = %ld, abs(end_y - start_y) = %ld\n", gesture_count, gesture_sum, abs(end_x - start_x), abs(end_y - start_y));
#endif
        if(gesture_count > 8) {
                if (gesture_is_o(gesture_count) && (support_gesture & TW_SUPPORT_O_SLIDE_WAKEUP))
                {
                		#if SMARTWAKE_DEBUG
                		pr_info(LOGTAG"------------GESTURE _O_ DETECTED----------");
                		#endif
                		sprintf(wakeup_slide,"o");
                        key_value = KEY_GESTURE_SLIDE_O;
                       
                }
        }
        if(gesture_count > 6 && key_value == 0) {
                if ((abs(end_x - start_x) > 40 && abs(end_y - start_y) > 40) && // start and end point should not be too close to each other
                        ((gesture_count == 9 && gesture_sum == RIGHT + RIGHT_UP + UP + LEFT_UP + LEFT + LEFT_DOWN + DOWN + RIGHT_DOWN + RIGHT) ||
                        (gesture_count == 8 && gesture_sum == RIGHT_UP + UP + LEFT_UP + LEFT + LEFT_DOWN + DOWN + RIGHT_DOWN + RIGHT) ||
                        (gesture_count == 8 && gesture_sum == RIGHT + RIGHT_UP + UP + LEFT_UP + LEFT + LEFT_DOWN + DOWN + RIGHT_DOWN) ||
                        (gesture_count == 7 && gesture_sum == RIGHT_UP + UP + LEFT_UP + LEFT + LEFT_DOWN + DOWN + RIGHT_DOWN)) && (support_gesture & TW_SUPPORT_E_SLIDE_WAKEUP))
                {	
                		#if SMARTWAKE_DEBUG	
                		pr_info(LOGTAG"------------GESTURE _E_ DETECTED----------");
                		#endif
                		sprintf(wakeup_slide,"e");
                        key_value = KEY_GESTURE_SLIDE_E;
                     
                } 

                else if (gesture_is_m(gesture_count) && (support_gesture & TW_SUPPORT_M_SLIDE_WAKEUP))
                {
                		#if SMARTWAKE_DEBUG
                		pr_info(LOGTAG"------------GESTURE _M_ DETECTED-----------");
                		#endif
                		sprintf(wakeup_slide,"m");
                        key_value = KEY_GESTURE_SLIDE_M;
                      

                }
        }
        if (key_value != 0 && exec_count) {
                smartwake_keytrigger();
                exec_count = false;
        }
        smartwake_reset();
}

/* Smartwake main function */
static void detect_smartwake(int x, int y)
{
        int delta_x = 0, delta_y = 0;
#if SMARTWAKE_DEBUG
        pr_info(LOGTAG"x,y(%4d,%4d) prev_x: %d\n",
                x, y, prev_x);
#endif       
        if (start_x == 0) start_x = x;
        if (start_y == 0) start_y = y;
        if (prev_x == 0) prev_x = x;
        if (prev_y == 0) prev_y = y;
        end_x = x;
        end_y = y;
        
        delta_x = x - prev_x;
        delta_y = prev_y - y;

        bool gradient = (abs(delta_y) < 2 * abs(delta_x) && 2 * abs(delta_y) > abs(delta_x));
                
        if (gradient && (delta_x * delta_x + delta_y * delta_y > MIN_DELTA * MIN_DELTA)) {
                if (delta_x > 0 && delta_y > 0)
                        direction[0] = RIGHT_UP;
                else if (delta_x > 0 && delta_y < 0)
                        direction[0] = RIGHT_DOWN;
                else if (delta_x < 0 && delta_y > 0)
                        direction[0] = LEFT_UP;
                else
                        direction[0] = LEFT_DOWN;
        } else if (abs(delta_x) > abs(delta_y) && abs(delta_x) > MIN_DELTA) {
                if (delta_x > 0)
                        direction[0] = RIGHT;
                else
                        direction[0] = LEFT;
        } else if (abs(delta_y) > MIN_DELTA) {
                if (delta_y > 0)
                        direction[0] = UP;
                else
                        direction[0] = DOWN;
        } else {
                direction[0] = 0;
        }
        direction_changed = direction[0] != 0 && old_direction != direction[0];

        prev_x = x;
        prev_y = y;

        if (direction_changed) {
                if (old_direction != 0) {
                        onlyGesture = true;
                }
                direction[1] = 0;
                direction[2] = 0;
#if SMARTWAKE_DEBUG
                pr_info(LOGTAG"direction[0] = %d, old_direction = %d\n", direction[0], old_direction);
#endif
                        
                if (((direction[0] > old_direction) && (direction[0] < 32 * old_direction)) || // direction < 32 * old one, because '>' is the other case
                        (direction[0] == 1 && old_direction == 64) ||
                        (direction[0] == 2 && old_direction == 128) ||
                        (32 * direction[0] == old_direction)) { // not clockwise
                        // if direction changed by 90 degree or 135 degree, add 45 degree and 90 degree
                        if (direction[0] == 8 * old_direction) {
                                direction[1] = 4 * old_direction;
                                direction[2] = 2 * old_direction;
                        } else if (direction[0] == 1 && old_direction == 32) {
                                direction[1] = 128;
                                direction[2] = 64;
                        } else if (direction[0] == 2 && old_direction == 64) {
                                direction[1] = 1;
                                direction[2] = 128;
                        } else if (direction[0] == 4 && old_direction == 128) {
                                direction[1] = 2;
                                direction[2] = 1;
                        } else if (direction[0] == 4 * old_direction) {
                                direction[1] = 2 * old_direction;
                        } else if (direction[0] == 1 && old_direction == 64) {
                                direction[1] = 128;
                        } else if (direction[0] == 2 && old_direction == 128) {
                                direction[1] = 1;
                        }
                } else if ((direction[0] < old_direction) || 
                        (old_direction == 1 && direction[0] == 64) ||
                        (old_direction == 2 && direction[0] == 128) ||
                        (direction[0] == 32 * old_direction)) { // clockwise
                        // if direction changed by 90 degree or 135 degree, add 45 degree and 90 degree
                        if (8 * direction[0] == old_direction) {
                                direction[1] = 2 * direction[0];
                                direction[2] = 4 * direction[0];
                        } else if (direction[0] == 32 && old_direction == 1) {
                                direction[1] = 64;
                                direction[2] = 128;
                        } else if (direction[0] == 64 && old_direction == 2) {
                                direction[1] = 128;
                                direction[2] = 1;
                        } else if (direction[0] == 128 && old_direction == 4) {
                                direction[1] = 1;
                                direction[2] = 2;                                
                        } else if (4 * direction[0] == old_direction) {
                                direction[1] = 2 * direction[0];
                        } else if (old_direction == 1 && direction[0] == 64) {
                                direction[1] = 128;
                        } else if (old_direction == 2 && direction[0] == 128) {
                                direction[1] = 1;
                        }
                }

                bool added_gesture = false;
                int i = 0, j = 0, int_count = 0;
                for (i = 0; i < 3; i++) {
                        if (direction[i] != 0) int_count = i;
                        else break;
                }
                
                for (i = 0; i + int_count < MAX_GESTURES; i++) {
                        if (passed_gestures[i] == 0) {
                                int added = 0;
                                for (j = 0; j <= int_count; j++) {
                                        if (passed_gestures[i + j] == 0) {
                                                passed_gestures[i + j] = direction[int_count - j];
#if SMARTWAKE_DEBUG
                                                pr_info(LOGTAG"direction[%d] = %d, i=%d\n", j, direction[j], i);
#endif
                                                added += 1;
                                        }
                                }
                                added_gesture = added == int_count + 1;
                        }
                        if (added_gesture) break;
                }
                if (added_gesture) {
                        for (i = 0; i < MAX_GESTURES; i++) {
                                if (x_i[i] == 0) { //y_i[i] should be 0 as well then....
                                        x_i[i] = x;
                                        y_i[i] = y;
                                        break;
                                }
                        }
                } else { // already tracked max number of gestures
                        pr_info(LOGTAG"Resetting: gesture not added");
                        smartwake_reset();
                        return;
                }   
        }
        
        if (!onlyGesture) {
                //Do nothing as we have removed vertical and horizontal gestures and added wake gestures from elementalx
        } else {
                onlyGesture = true;
        }
        if (direction[0] != 0) old_direction = direction[0];
}

static void smartwake_input_callback(struct work_struct *unused) {
#ifdef CONFIG_POCKETMOD
  	if (device_is_pocketed()){
  		return;
  	}
 	else
#endif	
        detect_smartwake(touch_x, touch_y);

        return;
}

static void smartwake_input_event(struct input_handle *handle, unsigned int type,
                                unsigned int code, int value) {
                                
if(!(smartwake_scr_suspended) || !smartwake_switch > 0)
        return;

	if (in_phone_call)
		return;

#if SMARTWAKE_DEBUG
        /*pr_info("smartwake: code: %s|%u, val: %i\n",
                ((code==ABS_MT_POSITION_X) ? "X" :
                (code==ABS_MT_POSITION_Y) ? "Y" :
                ((code==ABS_MT_TRACKING_ID)||
                        (code==330)) ? "ID" : "undef"), code, value);*/
#endif

        if (code == ABS_MT_TRACKING_ID) {
                if (finger_id == -1) {
                        finger_id = value;
                } else if (finger_id != value) {
#if SMARTWAKE_DEBUG
                        pr_info(LOGTAG"Resetting: ABS_MT_TRACKING_ID changed");
#endif
                        smartwake_reset();
                        return;
                }
        }
        
        if (code == ABS_MT_SLOT) {
#if SMARTWAKE_DEBUG
                pr_info(LOGTAG"Resetting: code == ABS_MT_SLOT");
#endif
                smartwake_reset();
                return;
        }

        /*
         * '330'? Many touch panels are 'broken' in the sense of not following the
         * multi-touch protocol given in Documentation/input/multi-touch-protocol.txt.
         * According to the docs, touch panels using the type B protocol must send in
         * a ABS_MT_TRACKING_ID event after lifting the contact in the first slot.
         * This should in the flow of events, help us reset the smartwake variables
         * and proceed as per the algorithm.
         *
         * This however is not the case with various touch panel drivers, and hence
         * there is no reliable way of tracking ABS_MT_TRACKING_ID on such panels.
         * Some of the panels however do track the lifting of contact, but with a
         * different event code, and a different event value.
         *
         * So, add checks for those event codes and values to keep the algo flow.
         *
         * synaptics_s3203 => code: 330; val: 0
         *
         * Note however that this is not possible with panels like the CYTTSP3 panel
         * where there are no such events being reported for the lifting of contacts
         * though i2c data has a ABS_MT_TRACKING_ID or equivalent event variable
         * present. In such a case, make sure the smartwake_reset() function is
         * publicly available for external calls.
         *
         */
        if ((code == ABS_MT_TRACKING_ID && value == -1) ||
                (code == 330 && value == 0)) {
#if SMARTWAKE_DEBUG
                pr_info(LOGTAG"checking gesture, finger lifted");
#endif
                check_gesture();
                return;
        }

        if (code == ABS_MT_POSITION_X) {
                touch_x = value;
                touch_x_called = true;
        }

        if (code == ABS_MT_POSITION_Y) {
                touch_y = value;
                touch_y_called = true;
        }

        if (touch_x_called && touch_y_called) {
                touch_x_called = false;
                touch_y_called = false;
                queue_work_on(0, smartwake_input_wq, &smartwake_input_work);
        }
}

static int input_dev_filter(struct input_dev *dev) {
        if (strstr(dev->name, "touch")||
                        strstr(dev->name, "mtk-tpd")) {
                return 0;
        } else {
                return 1;
        }
}

static int smartwake_input_connect(struct input_handler *handler,
                                struct input_dev *dev, const struct input_device_id *id) {
        struct input_handle *handle;
        int error;

        if (input_dev_filter(dev))
                return -ENODEV;

        handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
        if (!handle)
                return -ENOMEM;

        handle->dev = dev;
        handle->handler = handler;
        handle->name = "smartwake";
       
        error = input_register_handle(handle);
        if (error)
                goto err2;

        error = input_open_device(handle);
        if (error)
                goto err1;

        return 0;
err1:
        input_unregister_handle(handle);
err2:
        kfree(handle);
        return error;
}

static void smartwake_input_disconnect(struct input_handle *handle) {
        input_close_device(handle);
        input_unregister_handle(handle);
        kfree(handle);
}

static const struct input_device_id smartwake_ids[] = {
        { .driver_info = 1 },
        { },
};

static struct input_handler smartwake_input_handler = {
        .event          = smartwake_input_event,
        .connect        = smartwake_input_connect,
        .disconnect     = smartwake_input_disconnect,
        .name           = "smartwake_inputreq",
        .id_table       = smartwake_ids,
};

#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
static int lcd_notifier_callback(struct notifier_block *this,
                                unsigned long event, void *data)
{
        switch (event) {
        case LCD_EVENT_ON_END:
                smartwake_scr_suspended = false;
                break;
        case LCD_EVENT_OFF_END:
                smartwake_scr_suspended = true;
                break;
        default:
                break;
        }

        return 0;
}
#else
static void smartwake_early_suspend(struct early_suspend *h) {
        smartwake_scr_suspended = true;
}

static void smartwake_late_resume(struct early_suspend *h) {
        smartwake_scr_suspended = false;
}

static struct early_suspend smartwake_early_suspend_handler = {
        .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
        .suspend = smartwake_early_suspend,
        .resume = smartwake_late_resume,
};
#endif
#endif


//************* Sysfs control start **********************//

int goodix_active(void)
{
    pr_info(LOGTAG"Enter %s\n", __func__);
    return 1;
}

int goodix_get_wakeup_gesture(char*  gesture)
{
    return sprintf(gesture, "%s", (char *)wakeup_slide);
}

int goodix_get_gesture_ctrl(char*  gesture_ctrl)
{
    return sprintf(gesture_ctrl, "0x%08x", support_gesture);
}

int goodix_gesture_ctrl(const char*  gesture_buf)
{
    char *gesture;
    int buf_len;
    char *gesture_p;
    char *temp_p;
 
    buf_len = strlen(gesture_buf);

    pr_info(LOGTAG"%s buf_len = %d.\n", __func__, buf_len);
    pr_info(LOGTAG"%s gesture_buf:%s.\n", __func__, gesture_buf);

    gesture_p = kzalloc(buf_len + 1, GFP_KERNEL);
    if(gesture_p == NULL) {
        pr_err("%s: alloc mem error.\n", __func__);
        return -1;
    }
    temp_p = gesture_p;
    strlcpy(gesture_p, gesture_buf, buf_len + 1);

    while(gesture_p) {
        gesture = strsep(&gesture_p, ",");
        pr_info(LOGTAG"%s gesture:%s.\n", __func__, gesture);

        if (!strncmp(gesture, "e=",2)) {
            if(!strncmp(gesture+2, "true", 4)) {
                pr_info(LOGTAG"%s: enable e click wakeup func.\n", __func__);
                support_gesture |= TW_SUPPORT_E_SLIDE_WAKEUP;
            } else if(!strncmp(gesture+2, "false", 5)) {
                pr_info(LOGTAG"%s: disable e click wakeup func.\n", __func__);
                support_gesture &= ~TW_SUPPORT_E_SLIDE_WAKEUP;
            }
            continue;
        }

        if (!strncmp(gesture, "o=",2)) {
            if(!strncmp(gesture+2, "true", 4)) {
                pr_info(LOGTAG"%s: enable o click wakeup func.\n", __func__);
                support_gesture |= TW_SUPPORT_O_SLIDE_WAKEUP;
            } else if(!strncmp(gesture+2, "false", 5)) {
                pr_info(LOGTAG"%s: disable o click wakeup func.\n", __func__);
                support_gesture &= ~TW_SUPPORT_O_SLIDE_WAKEUP;
            }
            continue;
        }

        if (!strncmp(gesture, "m=",2)) {
            if(!strncmp(gesture+2, "true", 4)) {
                pr_info(LOGTAG"%s: enable m click wakeup func.\n", __func__);
                support_gesture |= TW_SUPPORT_M_SLIDE_WAKEUP;
            } else if(!strncmp(gesture+2, "false", 5)) {
                pr_info(LOGTAG"%s: disable m click wakeup func.\n", __func__);
                support_gesture &= ~TW_SUPPORT_M_SLIDE_WAKEUP;
            }
            continue;
        }
    }
    kfree(temp_p);
    return 1;
}

touchscreen_ops_tpye goodix_ops = {
		.touch_id = 0,
		.active 					= goodix_active,
        .get_wakeup_gesture         = goodix_get_wakeup_gesture,
        .get_gesture_ctrl           = goodix_get_gesture_ctrl,
        .gesture_ctrl               = goodix_gesture_ctrl,
};
//************* Sysfs control end **********************//


/*
 * INIT / EXIT stuff below here
 */
static int __init smartwake_init(void)
{
		int rc = 0;
        unsigned char attr_count;

        smartwake_pwrdev = input_allocate_device();
        if (!smartwake_pwrdev) {
                pr_err("Can't allocate suspend autotest power button\n");
                goto err_alloc_dev;
        }
	
     	for(attr_count=0; attr_count<TPD_GESTRUE_KEY_CNT; attr_count++)
        {
                input_set_capability(smartwake_pwrdev, EV_KEY, smartwake_gesture_keys[attr_count]);
        }


		smartwake_pwrdev->name = "smartwake_pwrkey";
		smartwake_pwrdev->phys = "smartwake_pwrkey/input0";

		touchscreen_set_ops(&goodix_ops);
       
        rc = input_register_device(smartwake_pwrdev);
        if (rc) {
                pr_err("%s: input_register_device err=%d\n", __func__, rc);
                goto err_input_dev;
        }

	smartwake_input_wq = create_workqueue("smartwakeiwq");
	if (!smartwake_input_wq) {
		pr_err("%s: Failed to create smartwakeiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&smartwake_input_work, smartwake_input_callback);
	rc = input_register_handler(&smartwake_input_handler);
	if (rc)
		pr_err("%s: Failed to register smartwake_input_handler\n", __func__);

#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
	smartwake_lcd_notif.notifier_call = lcd_notifier_callback;
	if (lcd_register_client(&smartwake_lcd_notif) != 0) {
		pr_err("%s: Failed to register lcd callback\n", __func__);
	}
#else
	register_early_suspend(&smartwake_early_suspend_handler);
#endif
#endif

err_input_dev:
        input_free_device(smartwake_pwrdev);
err_alloc_dev:
        pr_info(LOGTAG"%s done\n", __func__);
        return 0;
}

static void __exit smartwake_exit(void)
{

#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
        lcd_unregister_client(&smartwake_lcd_notif);
#endif
#endif
        input_unregister_handler(&smartwake_input_handler);
        destroy_workqueue(smartwake_input_wq);
        input_unregister_device(smartwake_pwrdev);
        input_free_device(smartwake_pwrdev);
        return;
}

module_init(smartwake_init);
module_exit(smartwake_exit);
