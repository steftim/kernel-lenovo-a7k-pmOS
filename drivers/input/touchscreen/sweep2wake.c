/*
 * drivers/input/touchscreen/sweep2wake.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2015, Vineeth Raj <contact.twn@openmailbox.org>
 * Copyright (c) 2013, Tanish <tanish2k09.dev@gmail.com>
 *
 * Wake Gestures
 * Copyright (c) 2014, Aaron Segaert <asegaert@gmail.com>
 *
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
#include <linux/input/sweep2wake.h>


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

/* uncomment since no touchscreen defines android touch, do that here */
//#define ANDROID_TOUCH_DECLARED

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "Tanish <tanish2k09.dev@gmail.com>"
#define DRIVER_DESCRIPTION "Swipe2wake for almost any device"
#define DRIVER_VERSION "2.0"
#define LOGTAG "[sweep2wake]: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

/* Tuneables */
#define S2W_DEBUG             0
#define S2W_DEFAULT           0
#define S2S_DEFAULT           0
#define S2W_PWRKEY_DUR      150
#define MIN_SWIPE_DISTANCE  400
#define VIB_STRENGTH 50

#define S2W_Y_MAX               1280
#define S2W_X_MAX               720
#define S2W_Y_LIMIT             S2W_Y_MAX-130
#define S2W_X_B1                250
#define S2W_X_B2                470
#define S2W_X_FINAL             120
#define S2W_Y_NEXT              160

/* Wake Gestures */
#define SWEEP_TIMEOUT		30
#define TRIGGER_TIMEOUT		50
#define WAKE_GESTURE		0x0b
#define WAKE_GESTURE_DEFAULT 0
#define SWEEP_RIGHT		0x01
#define SWEEP_LEFT		0x02
#define SWEEP_UP		0x04
#define SWEEP_DOWN		0x08

int gestures_switch = WAKE_GESTURE_DEFAULT;
int gestures_switch_cache = WAKE_GESTURE_DEFAULT;
static bool wake_gesture_file = WAKE_GESTURE_DEFAULT;
static struct input_dev *gesture_dev;

extern void set_vibrate(int value);

/* Resources */
extern void gestures_setdev(struct input_dev * input_device);
int s2w_switch = S2W_DEFAULT;
static int s2w_switch_cache = S2W_DEFAULT;
static bool s2w_file = S2W_DEFAULT;
static int s2s_switch = S2S_DEFAULT;
static int s2s_switch_cache = S2S_DEFAULT;
static bool s2s_file = S2S_DEFAULT;
unsigned int s2w_vibration_level = VIB_STRENGTH;
static int touch_x = 0, touch_y = 0;// init_x=-1, init_y=-1, init=-1;
static bool touch_x_called = false, touch_y_called = false;
//static bool exec_count = true;
bool s2w_scr_suspended = false;
static bool exec_countx = true, exec_county = true;
static bool barrierx[2] = {false, false}, barriery[2] = {false, false};
static int firstx = 0, firsty = 0;
static unsigned long firstx_time = 0, firsty_time = 0;
static unsigned long pwrtrigger_time[2] = {0, 0};


#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
static struct notifier_block s2w_lcd_notif;
#endif
#endif

static struct input_dev * sweep2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *s2w_input_wq;
static struct work_struct s2w_input_work;

/* PowerKey setter */
void sweep2wake_setdev(struct input_dev * input_device) {
	sweep2wake_pwrdev = input_device;
	printk(LOGTAG"set sweep2wake_pwrdev: %s\n", sweep2wake_pwrdev->name);
}

/* Read cmdline for s2w */
static int __init read_s2w_cmdline(char *s2w)
{
	if (strcmp(s2w, "1") == 0) {
		pr_info("[cmdline_s2w]: Sweep2Wake enabled. | s2w='%s'\n", s2w);
		s2w_switch = 1;
	} else if (strcmp(s2w, "0") == 0) {
		pr_info("[cmdline_s2w]: Sweep2Wake disabled. | s2w='%s'\n", s2w);
		s2w_switch = 0;
	} else {
		pr_info("[cmdline_s2w]: No valid input found. Going with default: | s2w='%u'\n", s2w_switch);
	}
	return 1;
}
__setup("s2w=", read_s2w_cmdline);


static void report_gesture(int gest)
{
        pwrtrigger_time[1] = pwrtrigger_time[0];
        pwrtrigger_time[0] = jiffies;	

	if (pwrtrigger_time[0] - pwrtrigger_time[1] < TRIGGER_TIMEOUT)
		return;

	pr_info(LOGTAG"gesture = %d\n", gest);
	input_report_rel(gesture_dev, WAKE_GESTURE, gest);
	input_sync(gesture_dev);
}



/* PowerKey work func */
static void sweep2wake_presspwr(struct work_struct * sweep2wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
		return;
	input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(sweep2wake_pwrdev, EV_SYN, 0, 0);
	msleep(S2W_PWRKEY_DUR);
	input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(sweep2wake_pwrdev, EV_SYN, 0, 0);
	msleep(S2W_PWRKEY_DUR);
	mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(sweep2wake_presspwr_work, sweep2wake_presspwr);

/* PowerKey trigger */
static void sweep2wake_pwrtrigger(void) {

    pwrtrigger_time[1] = pwrtrigger_time[0];
    pwrtrigger_time[0] = jiffies;
	
  	if (pwrtrigger_time[0] - pwrtrigger_time[1] < TRIGGER_TIMEOUT)
   		return;

    schedule_work(&sweep2wake_presspwr_work);
	return;
}

/* reset on finger release */
static void sweep2wake_reset(void) {
	exec_countx = true;
	barrierx[0] = false;
	barrierx[1] = false;
	firstx = 0;
	firstx_time = 0;

	exec_county = true;
	barriery[0] = false;
	barriery[1] = false;
	firsty = 0;
	firsty_time = 0;
}









/* Sweep2wake main function */
static void detect_sweep2wake_v(int x, int y, bool st)
{
	int prevy = 0, nexty = 0;
        bool single_touch = st;

	if (firsty == 0) {
		firsty = y;
		firsty_time = jiffies;
	}

	if (x > 70 && x < 650) {
		//up
		if (firsty > 640 && single_touch && (s2w_switch & SWEEP_UP)) {
			prevy = firsty;
			nexty = prevy - S2W_Y_NEXT;
			if (barriery[0] == true || (y < prevy && y > nexty)) {
				prevy = nexty;
				nexty -= S2W_Y_NEXT;
				barriery[0] = true;
				if (barriery[1] == true || (y < prevy && y > nexty)) {
					prevy = nexty;
					barriery[1] = true;
					if (y < prevy) {
						if (y < (nexty - S2W_Y_NEXT)) {
							if (exec_county && (jiffies - firsty_time < SWEEP_TIMEOUT)) {
								pr_info(LOGTAG"sweep up\n");
								set_vibrate(s2w_vibration_level);								
								if (gestures_switch) {
									report_gesture(3);
								} else {
						                        sweep2wake_pwrtrigger();
								}
								exec_county = false;
							}
						}
					}
				}
			}
		//down
		} else if (firsty <= 640 && single_touch && (s2w_switch & SWEEP_DOWN)) {
			prevy = firsty;
			nexty = prevy + S2W_Y_NEXT;
			if (barriery[0] == true || (y > prevy && y < nexty)) {
				prevy = nexty;
				nexty += S2W_Y_NEXT;
				barriery[0] = true;
				if (barriery[1] == true || (y > prevy && y < nexty)) {
					prevy = nexty;
					barriery[1] = true;
					if (y > prevy) {
						if (y > (nexty + S2W_Y_NEXT)) {
							if (exec_county && (jiffies - firsty_time < SWEEP_TIMEOUT)) {
								pr_info(LOGTAG"sweep down\n");								
								set_vibrate(s2w_vibration_level);
								if (gestures_switch) {
									report_gesture(4);
								} else {
						                        sweep2wake_pwrtrigger();
								}								
								exec_county = false;
							}
						}
					}
				}
			}
		}
	}
}









static void detect_sweep2wake_h(int x, int y, bool st, bool wake)
{
        int prevx = 0, nextx = 0;
        bool single_touch = st;

	if (firstx == 0) {
		firstx = x;
		firstx_time = jiffies;
	}

	if (!wake && y < S2W_Y_LIMIT) {
		sweep2wake_reset();
		return;
	}
#if S2W_DEBUG
        pr_info(LOGTAG"x,y(%4d,%4d) single:%s\n",
                x, y, (single_touch) ? "true" : "false");
#endif
	//left->right
	if (firstx < 250 && single_touch &&
		((wake && (s2w_switch & SWEEP_RIGHT)) || (!wake && (s2s_switch & SWEEP_RIGHT)))) {
		prevx = 0;
		nextx = S2W_X_B1;
		if ((barrierx[0] == true) ||
		   ((x > prevx) && (x < nextx))) {
			prevx = nextx;
			nextx = S2W_X_B2;
			barrierx[0] = true;
			if ((barrierx[1] == true) ||
			   ((x > prevx) && (x < nextx))) {
				prevx = nextx;
				barrierx[1] = true;
				if (x > prevx) {
					if (x > (S2W_X_MAX - S2W_X_FINAL)) {
						if (exec_countx && (jiffies - firstx_time < SWEEP_TIMEOUT)) {
							pr_info(LOGTAG"sweep right\n");		
							set_vibrate(s2w_vibration_level);					
							if (gestures_switch && wake) {
								report_gesture(1);
							} else {
						        	sweep2wake_pwrtrigger();
							}						
							exec_countx = false;
						}
					}
				}
			}
		}
	//right->left
	} else if (firstx >= 250 && single_touch &&
		((wake && (s2w_switch & SWEEP_LEFT)) || (!wake && (s2s_switch & SWEEP_LEFT)))) {
		prevx = (S2W_X_MAX - S2W_X_FINAL);
		nextx = S2W_X_B2;
		if ((barrierx[0] == true) ||
		   ((x < prevx) && (x > nextx))) {
			prevx = nextx;
			nextx = S2W_X_B1;
			barrierx[0] = true;
			if ((barrierx[1] == true) ||
			   ((x < prevx) && (x > nextx))) {
				prevx = nextx;
				barrierx[1] = true;
				if (x < prevx) {
					if (x < S2W_X_FINAL) {
						if (exec_countx) {
							pr_info(LOGTAG"sweep left\n");	
							set_vibrate(s2w_vibration_level);						
							if (gestures_switch && wake) {
								report_gesture(2);
							} else {
						        	sweep2wake_pwrtrigger();
							}							
							exec_countx = false;
						}
					}
				}
			}
		}
	}
}
















static void s2w_input_callback(struct work_struct *unused) {   
        
	detect_sweep2wake_h(touch_x, touch_y, true, s2w_scr_suspended);
	if (s2w_scr_suspended)
		detect_sweep2wake_v(touch_x, touch_y, true);

	return;
}

static void s2w_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
#if S2W_DEBUG
	pr_info("sweep2wake: code: %s|%u, val: %i\n",
		((code==ABS_MT_POSITION_X) ? "X" :
		(code==ABS_MT_POSITION_Y) ? "Y" :
		((code==ABS_MT_TRACKING_ID)||
			(code==330)) ? "ID" : "undef"), code, value);
#endif


	if (code == ABS_MT_SLOT) {
		sweep2wake_reset();
		return;
	}

	/*
	 * '330'? Many touch panels are 'broken' in the sense of not following the
	 * multi-touch protocol given in Documentation/input/multi-touch-protocol.txt.
	 * According to the docs, touch panels using the type B protocol must send in
	 * a ABS_MT_TRACKING_ID event after lifting the contact in the first slot.
	 * This should in the flow of events, help us reset the sweep2wake variables
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
	 * present. In such a case, make sure the sweep2wake_reset() function is
	 * publicly available for external calls.
	 *
	 */
	if ((code == ABS_MT_TRACKING_ID && value == -1) ||
		(code == 330 && value == 0)) {
		sweep2wake_reset();
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
		queue_work_on(0, s2w_input_wq, &s2w_input_work);
	} else if (!s2w_scr_suspended && touch_x_called && !touch_y_called) {
		touch_x_called = false;
		touch_y_called = false;
		queue_work_on(0, s2w_input_wq, &s2w_input_work);
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

static int s2w_input_connect(struct input_handler *handler,
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
	handle->name = "s2w";

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

static void s2w_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id s2w_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler s2w_input_handler = {
	.event		= s2w_input_event,
	.connect	= s2w_input_connect,
	.disconnect	= s2w_input_disconnect,
	.name		= "s2w_inputreq",
	.id_table	= s2w_ids,
};

#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
static int lcd_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	switch (event) {
	case LCD_EVENT_ON_END:
		s2w_scr_suspended = false;
		break;
	case LCD_EVENT_OFF_END:
		s2w_scr_suspended = true;
		break;
	default:
		break;
	}

	return 0;
}
#else
static void s2w_early_suspend(struct early_suspend *h) {
	s2w_scr_suspended = true;
}

static void s2w_late_resume(struct early_suspend *h) {
	s2w_scr_suspended = false;
}

static struct early_suspend s2w_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = s2w_early_suspend,
	.resume = s2w_late_resume,
};
#endif
#endif

/*
 * SYSFS stuff below here
 */

static ssize_t s2w_sweep2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch_cache);

	return count;
}

static ssize_t s2w_sweep2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_cache);
	if (s2w_switch_cache < 0 || s2w_switch_cache > 15)
		s2w_switch_cache = 15;

	return count;
}


static DEVICE_ATTR(sweep2wake, (S_IWUSR|S_IRUGO),
	s2w_sweep2wake_show, s2w_sweep2wake_dump);


static ssize_t s2w_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%s\n", DRIVER_VERSION);

	return count;
}

static ssize_t s2w_version_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(sweep2wake_version, (S_IWUSR|S_IRUGO),
	s2w_version_show, s2w_version_dump);
	
	
static ssize_t sweep2sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", s2s_switch_cache);
	return count;
}

static ssize_t sweep2sleep_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '3' && buf[1] == '\n')
                if (s2s_switch_cache != buf[0] - '0')
		        s2s_switch_cache = buf[0] - '0';
	return count;
}

static DEVICE_ATTR(sweep2sleep, (S_IWUSR|S_IRUGO),
	sweep2sleep_show, sweep2sleep_dump);


static ssize_t wake_gestures_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", gestures_switch_cache);
	return count;
}

static ssize_t wake_gestures_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
                if (gestures_switch_cache != buf[0] - '0')
		        gestures_switch_cache = buf[0] - '0';
	return count;
}

static DEVICE_ATTR(wake_gestures, (S_IWUSR|S_IRUGO),
	wake_gestures_show, wake_gestures_dump);


/*
 * INIT / EXIT stuff below here
 */
#ifdef ANDROID_TOUCH_DECLARED
extern struct kobject *android_touch_kobj;
#else
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);
#endif

void wakeg_sysfs_all(bool w)
{    
    if (w)
    {
        int rc = 0;
        gestures_switch = gestures_switch_cache;
        s2w_switch = s2w_switch_cache;
        s2s_switch = s2s_switch_cache;
        if (!wake_gesture_file)
        {
            rc = sysfs_create_file(android_touch_kobj, &dev_attr_wake_gestures.attr);
            if (rc) 
            {
	            pr_warn("%s: sysfs_create_file failed for wake_gestures\n", __func__);
            }
            else
                wake_gesture_file = 1;    
        }
        if (!s2w_file)
        {
            rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake.attr);
	        if (rc) {
		        pr_warn("%s: sysfs_create_file failed for sweep2wake\n", __func__);
	        }
	        else
	            s2w_file = 1;
	    }
	    if (!s2s_file)
	    {
	        rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2sleep.attr);
	        if (rc) {
		        pr_warn("%s: sysfs_create_file failed for sweep2sleep\n", __func__);
	        }
	        else 
    	        s2s_file = 1;
	    }     
    }
    else
    {
        gestures_switch_cache = gestures_switch;
        gestures_switch = 0;
        s2w_switch_cache = s2w_switch;
        s2w_switch = 0;
        s2s_switch_cache = s2s_switch;
        s2s_switch = 0;
        
        if (wake_gesture_file)
            sysfs_remove_file(android_touch_kobj, &dev_attr_wake_gestures.attr);
        if (s2w_file)    
            sysfs_remove_file(android_touch_kobj, &dev_attr_sweep2wake.attr);
        if (s2s_file)
            sysfs_remove_file(android_touch_kobj, &dev_attr_sweep2sleep.attr);
        wake_gesture_file = 0;        
        s2w_file = 0;
        s2s_file = 0;
    }
}


static int __init sweep2wake_init(void)
{
	int rc = 0;

	s2w_input_wq = create_workqueue("s2wiwq");
	if (!s2w_input_wq) {
		pr_err("%s: Failed to create s2wiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&s2w_input_work, s2w_input_callback);
	rc = input_register_handler(&s2w_input_handler);
	if (rc)
		pr_err("%s: Failed to register s2w_input_handler\n", __func__);
		
		
    gesture_dev = input_allocate_device();
	if (!gesture_dev) {
		goto err_alloc_dev;
	}

	gesture_dev->name = "wake_gesture";
	gesture_dev->phys = "wake_gesture/input1";
	input_set_capability(gesture_dev, EV_REL, WAKE_GESTURE);

	rc = input_register_device(gesture_dev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}
	gestures_setdev(gesture_dev);

		

#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
	s2w_lcd_notif.notifier_call = lcd_notifier_callback;
	if (lcd_register_client(&s2w_lcd_notif) != 0) {
		pr_err("%s: Failed to register lcd callback\n", __func__);
	}
#else
	register_early_suspend(&s2w_early_suspend_handler);
#endif
#endif

#ifndef ANDROID_TOUCH_DECLARED
	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
	}
#endif
    if (S2W_DEFAULT)
    {
	    rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake.attr);
	    if (rc) {
		    pr_warn("%s: sysfs_create_file failed for sweep2wake\n", __func__);
	    }
	    else
	        s2w_file = 1;
	    rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_version.attr);
	    if (rc) {
		    pr_warn("%s: sysfs_create_file failed for sweep2wake_version\n", __func__);
	    }
	}
	else
	    s2w_file = 0;
	    
	if (S2S_DEFAULT)
	{
	    rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2sleep.attr);
	    if (rc) {
		    pr_warn("%s: sysfs_create_file failed for sweep2sleep\n", __func__);
	    }
	    else
	        s2s_file = 1;	        
	}
	else
	    s2s_file = 0;
	    
	if (WAKE_GESTURE_DEFAULT)
	{
	    rc = sysfs_create_file(android_touch_kobj, &dev_attr_wake_gestures.attr);
	    if (rc) {
		    pr_warn("%s: sysfs_create_file failed for wake_gestures\n", __func__);
	    }
	    else 
	        wake_gesture_file = 1;
	}
	else
	    wake_gesture_file = 0;
	    
err_input_dev:
	input_free_device(sweep2wake_pwrdev);
err_alloc_dev:
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit sweep2wake_exit(void)
{
#ifndef ANDROID_TOUCH_DECLARED
	kobject_del(android_touch_kobj);
#endif
#ifndef WAKE_HOOKS_DEFINED
#ifndef CONFIG_HAS_EARLYSUSPEND
	lcd_unregister_client(&s2w_lcd_notif);
#endif
#endif
	input_unregister_handler(&s2w_input_handler);
	destroy_workqueue(s2w_input_wq);
	return;
}

module_init(sweep2wake_init);
module_exit(sweep2wake_exit);
