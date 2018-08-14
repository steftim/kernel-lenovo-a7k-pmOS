/*
 * drivers/cpufreq/cpufreq_poison.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2015 Varun Chitre.
 * Copyright (C) 2015 tanish2k09.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: tanish2k09 (tanish2k09@gmail.com) (tanish2k09@xda-developers.com)
 *
 * Based on the thunderx governor By Erasmux
 * which was adaptated to 2.6.29 kernel by Nadlabak (pavel@doshaska.net)
 *
 * SMP support based on mod by faux123
 * Mediatek Soc support by varunchitre15
 *
 */

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <asm/cputime.h>
#include <linux/earlysuspend.h>
#include <linux/input/doubletap2wake.h>
#include <linux/input/trace2wake.h>
#include "../drivers/misc/mediatek/connectivity/conn_soc/drv_wlan/mt_wifi/wlan/include/wlan_pms.h"
#include "../drivers/misc/mediatek/video/klapse/klapse_ext_ref.h"
#include <linux/pocket_mod.h>
#include "../drivers/power/mediatek/fastchg.h"

#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE
#include <linux/input/sweep2wake.h>
#include "../drivers/input/touchscreen/mediatek/GT9XX_aio/smartwake.h"
#endif

/******************** Tunable parameters: ********************/

/*
 * The "ideal" frequency to use when awake. The governor will ramp up faster
 * towards the ideal frequency and slower after it has passed it. Similarly,
 * lowering the frequency towards the ideal frequency is faster than below it.
 */
#define DEFAULT_AWAKE_IDEAL_FREQ 1170000
static unsigned int awake_ideal_freq;

/*
 * The "ideal" frequency to use when suspended.
 * When set to 0, the governor will not track the suspended state (meaning
 * that practically when sleep_ideal_freq==0 the awake_ideal_freq is used
 * also when suspended).
 */
#define DEFAULT_SLEEP_IDEAL_FREQ 468000
static unsigned int sleep_ideal_freq;

/*
 * Frequency delta when ramping up above the ideal freqeuncy.
 * Zero disables and causes to always jump straight to max frequency.
 * When below the ideal freqeuncy we always ramp up to the ideal freq.
 */
#define DEFAULT_RAMP_UP_STEP 70000
static unsigned int ramp_up_step;

/*
 * Frequency delta when ramping down below the ideal freqeuncy.
 * Zero disables and will calculate ramp down according to load heuristic.
 * When above the ideal freqeuncy we always ramp down to the ideal freq.
 */
#define DEFAULT_RAMP_DOWN_STEP 80000
static unsigned int ramp_down_step;

/*
 * CPU freq will be increased if measured load > max_cpu_load;
 */
#define DEFAULT_MAX_CPU_LOAD 75
static unsigned long max_cpu_load;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
#define DEFAULT_MIN_CPU_LOAD 40
static unsigned long min_cpu_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp up.
 * Notice we ignore this when we are below the ideal frequency.
 */
#define DEFAULT_UP_RATE_US 10000;
static unsigned long up_rate_us;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 * Notice we ignore this when we are above the ideal frequency.
 */
#define DEFAULT_DOWN_RATE_US 80000;
static unsigned long down_rate_us;

/*
 * The frequency to set when waking up from sleep.
 * When sleep_ideal_freq=0 this will have no effect.
 */
#define DEFAULT_SLEEP_WAKEUP_FREQ 1417000
static unsigned int sleep_wakeup_freq;

/*
 * Sampling rate, highly recommended to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

/*
 ******* Custom tunables for A7000 (aio_row) ********
 */

static unsigned int dt2w_vib, x_left, x_right, y_up, y_down;

#ifdef CONFIG_TOUCHSCREEN_PREVENT_SLEEP
/*#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
extern void dt2w_sysfs_all(bool d);
#endif*/
#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE
static unsigned int gesture_min_x, gesture_min_y, gesture_vib, smartwake_overrides_customgestures;
extern void wakeg_sysfs_all(bool w);
extern int gestures_switch;
#endif
#ifdef CONFIG_TOUCHSCREEN_SWEEP2WAKE
static unsigned int s2w_vibe;
extern void smartwake_sysfs_toggle(bool s);
#endif
#endif

static unsigned int dt2w_tap_distance, dt2w_time_for_tap, dt2w_on_buttons_enabled;
static bool pocket_mod_enabled, revert_dt2w_area_bounded_by_xy;
static int WificustPowerMode, ac_charge_level, usb_charge_level, enable_trace2sleep;
static unsigned int vib_power_in_pocket_mod_dt2w_override, pocket_mode_dt2w_override_taps_timeout, pocket_mode_dt2w_override_taps_number;
static bool gaming_mode = 0, do_boost = 0;
extern void set_override(bool override);

//////////////////////////////////////

bool no_vibrate=false;
bool vibrator_on_lock = false;
bool force_disable_gestures_HARD = false;
bool force_enable_gestures_charging = 0;


/*************** End of tunables ***************/


static atomic_t active_count = ATOMIC_INIT(0);

struct thunderx_info_s {
	struct cpufreq_policy *cur_policy;
	struct cpufreq_frequency_table *freq_table;
	struct timer_list timer;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 freq_change_time;
	u64 freq_change_time_in_idle;
	int cur_cpu_load;
	int old_freq;
	int ramp_dir;
	unsigned int enable;
	int ideal_speed;
};
static DEFINE_PER_CPU(struct thunderx_info_s, thunderx_info);

/* Workqueues handle frequency scaling */
static struct workqueue_struct *up_wq;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_work;

static cpumask_t work_cpumask;
static spinlock_t cpumask_lock;

static unsigned int suspended;

#define dprintk(flag,msg...) do { \
	if (debug_mask & flag) printk(KERN_DEBUG msg); \
	} while (0)

enum {
	THUNDERX_DEBUG_JUMPS=1,
	THUNDERX_DEBUG_LOAD=2,
	THUNDERX_DEBUG_ALG=4
};

/*
 * Combination of the above debug flags.
 */
static unsigned long debug_mask;

static int cpufreq_governor_thunderx(struct cpufreq_policy *policy,
		unsigned int event);

#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)
#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_POISON
static
#endif
struct cpufreq_governor cpufreq_gov_poison = {
	.name = "Poison",
	.governor = cpufreq_governor_thunderx,
	.max_transition_latency = TRANSITION_LATENCY_LIMIT,
	.owner = THIS_MODULE,
};






inline static void thunderx_update_min_max(struct thunderx_info_s *this_thunderx, struct cpufreq_policy *policy, int suspend) {
	if (suspend) {
        no_vibrate=true;
		if(gaming_mode)
		{
			do_boost = 0; //If gaming mode is on, stop cpu boost on suspend. Prevent battery and heat.			
	    }
		this_thunderx->ideal_speed = // sleep_ideal_freq; but make sure it obeys the policy min/max
			policy->max > sleep_ideal_freq ?
			(sleep_ideal_freq > policy->min ? sleep_ideal_freq : policy->min) : policy->max;
	} else {
        no_vibrate=false;		
		if(gaming_mode)
		{
			do_boost = 1; //If gaming mode is on, boost the clock on resume.
		}
		this_thunderx->ideal_speed = // awake_ideal_freq; but make sure it obeys the policy min/max
			policy->min < awake_ideal_freq ?
			(awake_ideal_freq < policy->max ? awake_ideal_freq : policy->max) : policy->min;
	}
	
	set_override(do_boost);

            
            /////////////////////////////// 
}

inline static void thunderx_update_min_max_allcpus(void) {
	unsigned int i;
	for_each_online_cpu(i) {
		struct thunderx_info_s *this_thunderx = &per_cpu(thunderx_info, i);
		if (this_thunderx->enable)
			thunderx_update_min_max(this_thunderx,this_thunderx->cur_policy,suspended);
	}
}

inline static unsigned int validate_freq(struct cpufreq_policy *policy, int freq) {

	if (do_boost) 
	{
	    if(freq < 1495000)
		    freq = 1495000; //Handle gaming mode freq scaling. Bool gaming_mode will switch according to suspend state before coming to this function. due to the fact that it is set in update_min_max function :D
		else if (freq < 1560000)
		    freq = 1560000;
		else if (freq < 1690000)
		    freq = 1690000;
    }		      
						
			
	if (freq > (int)policy->max)
		return policy->max;
	if (freq < (int)policy->min)
		return policy->min;	 
	return freq;

}

inline static void reset_timer(unsigned long cpu, struct thunderx_info_s *this_thunderx) {
	this_thunderx->time_in_idle = get_cpu_idle_time_us(cpu, &this_thunderx->idle_exit_time);
	mod_timer(&this_thunderx->timer, jiffies + sample_rate_jiffies);
}

inline static void work_cpumask_set(unsigned long cpu) {
	unsigned long flags;
	spin_lock_irqsave(&cpumask_lock, flags);
	cpumask_set_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
}

inline static int work_cpumask_test_and_clear(unsigned long cpu) {
	unsigned long flags;
	int res = 0;
	spin_lock_irqsave(&cpumask_lock, flags);
	res = cpumask_test_and_clear_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
	return res;
}

inline static int target_freq(struct cpufreq_policy *policy, struct thunderx_info_s *this_thunderx,
			      int new_freq, int old_freq, int prefered_relation) {
	int index, target;
	struct cpufreq_frequency_table *table = this_thunderx->freq_table;

	if (new_freq == old_freq)
		return 0;
	new_freq = validate_freq(policy,new_freq);
	if (new_freq == old_freq)
		return 0;

    //Overclock scaling handling
    if ((new_freq > 1495000) && (old_freq < 1495000))
    {
        if (policy->max < 1495000)
            new_freq = policy->max;
        else
            new_freq = 1495000;
    }
    
    if ((new_freq > 1570000) && (old_freq < 1550000))
    {
        new_freq = 1560000;
    }   
    //Overclock handling finish 

	if (table &&
	    !cpufreq_frequency_table_target(policy,table,new_freq,prefered_relation,&index))
	{
		target = table[index].frequency;
		if (target == old_freq) {
			// if for example we are ramping up to *at most* current + ramp_up_step
			// but there is no such frequency higher than the current, try also
			// to ramp up to *at least* current + ramp_up_step.
			if (new_freq > old_freq && prefered_relation==CPUFREQ_RELATION_H
			    && !cpufreq_frequency_table_target(policy,table,new_freq,
							       CPUFREQ_RELATION_L,&index))
				target = table[index].frequency;
			// simlarly for ramping down:
			else if (new_freq < old_freq && prefered_relation==CPUFREQ_RELATION_L
				&& !cpufreq_frequency_table_target(policy,table,new_freq,
								   CPUFREQ_RELATION_H,&index))
				target = table[index].frequency;
		}

		if (target == old_freq) {
			// We should not get here:
			// If we got here we tried to change to a validated new_freq which is different
			// from old_freq, so there is no reason for us to remain at same frequency.
			printk(KERN_WARNING "thunderX: frequency change failed: %d to %d => %d\n",
			       old_freq,new_freq,target);
			return 0;
		}
	}
	else target = new_freq;

	__cpufreq_driver_target(policy, target, prefered_relation);

	dprintk(THUNDERX_DEBUG_JUMPS,"thunderX: jumping from %d to %d => %d (%d)\n",
		old_freq,new_freq,target,policy->cur);

	return target;
}

static void cpufreq_thunderx_timer(unsigned long cpu)
{
	u64 delta_idle;
	u64 delta_time;
	int cpu_load;
	int old_freq;
	u64 update_time;
	u64 now_idle;
	int queued_work = 0;
	struct thunderx_info_s *this_thunderx = &per_cpu(thunderx_info, cpu);
	struct cpufreq_policy *policy = this_thunderx->cur_policy;

	now_idle = get_cpu_idle_time_us(cpu, &update_time);
	old_freq = policy->cur;

	if (this_thunderx->idle_exit_time == 0 || update_time == this_thunderx->idle_exit_time)
		return;

	delta_idle = (now_idle - this_thunderx->time_in_idle);
    delta_time = (update_time - this_thunderx->idle_exit_time);

	// If timer ran less than 1ms after short-term sample started, retry.
	if (delta_time < 1000) {
		if (!timer_pending(&this_thunderx->timer))
			reset_timer(cpu,this_thunderx);
		return;
	}

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (unsigned int)(delta_time - delta_idle) / (unsigned int)delta_time;

	dprintk(THUNDERX_DEBUG_LOAD,"thunderxT @ %d: load %d (delta_time %llu)\n",
		old_freq,cpu_load,delta_time);

	this_thunderx->cur_cpu_load = cpu_load;
	this_thunderx->old_freq = old_freq;

	// Scale up if load is above max or if there where no idle cycles since coming out of idle,
	// additionally, if we are at or above the ideal_speed, verify we have been at this frequency
	// for at least up_rate_us:
	if (cpu_load > max_cpu_load || delta_idle == 0)
	{
		if (old_freq < policy->max &&
			 (old_freq < this_thunderx->ideal_speed || delta_idle == 0 ||
			  (update_time - this_thunderx->freq_change_time) >= up_rate_us))
		{
			dprintk(THUNDERX_DEBUG_ALG,"thunderxT @ %d ramp up: load %d (delta_idle %llu)\n",
				old_freq,cpu_load,delta_idle);
			this_thunderx->ramp_dir = 1;
			work_cpumask_set(cpu);
			queue_work(up_wq, &freq_scale_work);
			queued_work = 1;
		}
		else this_thunderx->ramp_dir = 0;
	}
	// Similarly for scale down: load should be below min and if we are at or below ideal
	// frequency we require that we have been at this frequency for at least down_rate_us:
	else if (cpu_load < min_cpu_load && old_freq > policy->min &&
		 (old_freq > this_thunderx->ideal_speed ||
		  (update_time - this_thunderx->freq_change_time) >= down_rate_us))
	{
		dprintk(THUNDERX_DEBUG_ALG,"thunderxT @ %d ramp down: load %d (delta_idle %llu)\n",
			old_freq,cpu_load,delta_idle);
		this_thunderx->ramp_dir = -1;
		work_cpumask_set(cpu);
		queue_work(down_wq, &freq_scale_work);
		queued_work = 1;
	}
	else this_thunderx->ramp_dir = 0;

	// To avoid unnecessary load when the CPU is already at high load, we don't
	// reset ourselves if we are at max speed. If and when there are idle cycles,
	// the idle loop will activate the timer.
	// Additionally, if we queued some work, the work task will reset the timer
	// after it has done its adjustments.
	if (!queued_work && !suspended)
		reset_timer(cpu,this_thunderx);
}

/* We use the same work function to sale up and down */
static void cpufreq_thunderx_freq_change_time_work(struct work_struct *work)
{
	unsigned int cpu;
	int new_freq;
	int old_freq;
	int ramp_dir;
	struct thunderx_info_s *this_thunderx;
	struct cpufreq_policy *policy;
	unsigned int relation = CPUFREQ_RELATION_L;
	for_each_possible_cpu(cpu) {
		this_thunderx = &per_cpu(thunderx_info, cpu);
		if (!work_cpumask_test_and_clear(cpu))
			continue;

		ramp_dir = this_thunderx->ramp_dir;
		this_thunderx->ramp_dir = 0;

		old_freq = this_thunderx->old_freq;
		policy = this_thunderx->cur_policy;

		if (old_freq != policy->cur) {
			// frequency was changed by someone else?
			printk(KERN_WARNING "thunderX: frequency changed by 3rd party: %d to %d\n",
			       old_freq,policy->cur);
			new_freq = old_freq;
		}
		else if (ramp_dir > 0 && nr_running() > 1) {
			// ramp up logic:
			if (old_freq < this_thunderx->ideal_speed)
				new_freq = this_thunderx->ideal_speed;
			else if (ramp_up_step) {
				new_freq = old_freq + ramp_up_step;
				relation = CPUFREQ_RELATION_H;
			}
			else {
				new_freq = policy->max;
				relation = CPUFREQ_RELATION_H;
			}
			dprintk(THUNDERX_DEBUG_ALG,"thunderxQ @ %d ramp up: ramp_dir=%d ideal=%d\n",
				old_freq,ramp_dir,this_thunderx->ideal_speed);
		}
		else if (ramp_dir < 0) {
			// ramp down logic:
			if (old_freq > this_thunderx->ideal_speed) {
				new_freq = this_thunderx->ideal_speed;
				relation = CPUFREQ_RELATION_H;
			}
			else if (ramp_down_step)
				new_freq = old_freq - ramp_down_step;
			else {
				// Load heuristics: Adjust new_freq such that, assuming a linear
				// scaling of load vs. frequency, the load in the new frequency
				// will be max_cpu_load:
				new_freq = old_freq * this_thunderx->cur_cpu_load / max_cpu_load;
				if (new_freq > old_freq) // min_cpu_load > max_cpu_load ?!
					new_freq = old_freq -1;
			}
			dprintk(THUNDERX_DEBUG_ALG,"thunderxQ @ %d ramp down: ramp_dir=%d ideal=%d\n",
				old_freq,ramp_dir,this_thunderx->ideal_speed);
		}
		else { // ramp_dir==0 ?! Could the timer change its mind about a queued ramp up/down
		       // before the work task gets to run?
		       // This may also happen if we refused to ramp up because the nr_running()==1
			new_freq = old_freq;
			dprintk(THUNDERX_DEBUG_ALG,"thunderxQ @ %d nothing: ramp_dir=%d nr_running=%lu\n",
				old_freq,ramp_dir,nr_running());
		}

		// do actual ramp up (returns 0, if frequency change failed):
		new_freq = target_freq(policy,this_thunderx,new_freq,old_freq,relation);
		if (new_freq)
			this_thunderx->freq_change_time_in_idle =
				get_cpu_idle_time_us(cpu,&this_thunderx->freq_change_time);

		// reset timer:
		if (new_freq < policy->max)
			reset_timer(cpu,this_thunderx);
		// if we are maxed out, it is pointless to use the timer
		// (idle cycles wake up the timer when the timer comes)
		else if (timer_pending(&this_thunderx->timer))
			del_timer(&this_thunderx->timer);
	}
}

static ssize_t show_debug_mask(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", debug_mask);
}

static ssize_t store_debug_mask(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0)
		debug_mask = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_up_rate_us(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", up_rate_us);
}

static ssize_t store_up_rate_us(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 100000000)
		up_rate_us = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_down_rate_us(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", down_rate_us);
}

static ssize_t store_down_rate_us(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 100000000)
		down_rate_us = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_sleep_ideal_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sleep_ideal_freq);
}

static ssize_t store_sleep_ideal_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		sleep_ideal_freq = input;
		if (suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_sleep_wakeup_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sleep_wakeup_freq);
}

static ssize_t store_sleep_wakeup_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		sleep_wakeup_freq = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_awake_ideal_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", awake_ideal_freq);
}

static ssize_t store_awake_ideal_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		awake_ideal_freq = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_sample_rate_jiffies(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sample_rate_jiffies);
}

static ssize_t store_sample_rate_jiffies(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input <= 1000)
		sample_rate_jiffies = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_ramp_up_step(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ramp_up_step);
}

static ssize_t store_ramp_up_step(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		ramp_up_step = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_ramp_down_step(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ramp_down_step);
}

static ssize_t store_ramp_down_step(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		ramp_down_step = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_max_cpu_load(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", max_cpu_load);
}

static ssize_t store_max_cpu_load(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input <= 100)
		max_cpu_load = input;
	else return -EINVAL;
	return count;
}

static ssize_t show_min_cpu_load(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_cpu_load);
}

static ssize_t store_min_cpu_load(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input < 100)
		min_cpu_load = input;
	else return -EINVAL;
	return count;
}

////////////////////////////////////////////////////////////////////////////////////////////////
/* Now here we shall declare the functions to show and store custom tunables for A7000 (aio_row) */
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
///////////////This just for noticing lol////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////




// This is the dt2w area part :

static ssize_t show_x_left(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", x_left);
}

static ssize_t store_x_left(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
            if(input>=x_right)
                return count;
		x_left = input;
        left = x_left;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_x_right(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", x_right);
}

static ssize_t store_x_right(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
            if(input>720 || input<=x_left)
                return count;
		x_right = input;
        right = x_right;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_y_up(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", y_up);
}

static ssize_t store_y_up(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
            if(input>=y_down)
                return count;
		y_up = input;
        co_up = y_up;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_y_down(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", y_down);
}

static ssize_t store_y_down(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
            if(input>1280 || input<=y_up)
                return count;
		y_down = input;
        co_down = y_down;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}


/////////////////////////////////////////////////////////////////////////

static ssize_t show_dt2w_time_for_tap(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dt2w_time_for_tap);
}

static ssize_t store_dt2w_time_for_tap(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0) {

		dt2w_time_for_tap = input;
        dt2w_time = dt2w_time_for_tap;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_dt2w_tap_distance(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dt2w_tap_distance);
}

static ssize_t store_dt2w_tap_distance(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0) {

		dt2w_tap_distance = input;
        dt2w_radius = dt2w_tap_distance;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}


static ssize_t show_ac_charge_level(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ac_charge_level);
}

static ssize_t store_ac_charge_level(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= AC_CHARGE_LEVEL_MIN && input <= AC_CHARGE_LEVEL_MAX) {
		ac_charge_level = input;
        ac_level = ac_charge_level;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_usb_charge_level(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", usb_charge_level);
}

static ssize_t store_usb_charge_level(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= USB_CHARGE_LEVEL_MIN && input <= USB_CHARGE_LEVEL_MAX) {
		usb_charge_level = input;
        usb_level = usb_charge_level;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}


static ssize_t show_revert_dt2w_area_bounded_by_xy(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", revert_dt2w_area_bounded_by_xy);
}

static ssize_t store_revert_dt2w_area_bounded_by_xy(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input<=1) {
		revert_dt2w_area_bounded_by_xy = input;
        revert_area = revert_dt2w_area_bounded_by_xy;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_pocket_mod_enabled(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pocket_mod_enabled);
}

static ssize_t store_pocket_mod_enabled(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input<=1) {
		pocket_mod_enabled = input;
        pocket_mod_switch = pocket_mod_enabled;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}


static ssize_t show_dt2w_vib(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dt2w_vib);
}

static ssize_t store_dt2w_vib(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		dt2w_vib = input;
        vib_strength = dt2w_vib;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_WificustPowerMode(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", WificustPowerMode);
}

static ssize_t store_WificustPowerMode(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 2) {
		WificustPowerMode = input;
        custPowerMode = WificustPowerMode;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_dt2w_on_buttons_enabled(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dt2w_on_buttons_enabled);
}

static ssize_t store_dt2w_on_buttons_enabled(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input<=2) {
		dt2w_on_buttons_enabled = input;
        Dt2w_regions = dt2w_on_buttons_enabled;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_vibrator_on_lock(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", vibrator_on_lock);
}

static ssize_t store_vibrator_on_lock(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input<=1) {
		vibrator_on_lock = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_vib_power_in_pocket_mod_dt2w_override(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", vib_power_in_pocket_mod_dt2w_override);
}

static ssize_t store_vib_power_in_pocket_mod_dt2w_override(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		vib_power_in_pocket_mod_dt2w_override = input;
        vibration_strength_on_pocket_override = vib_power_in_pocket_mod_dt2w_override;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_pocket_mode_dt2w_override_taps_timeout(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pocket_mode_dt2w_override_taps_timeout);
}

static ssize_t store_pocket_mode_dt2w_override_taps_timeout(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		pocket_mode_dt2w_override_taps_timeout = input;
        pocket_override_timeout = pocket_mode_dt2w_override_taps_timeout;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_pocket_mode_dt2w_override_taps_number(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pocket_mode_dt2w_override_taps_number);
}

static ssize_t store_pocket_mode_dt2w_override_taps_number(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 3) {
		pocket_mode_dt2w_override_taps_number = input;
        dt2w_override_taps = pocket_mode_dt2w_override_taps_number;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_enable_trace2sleep(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", enable_trace2sleep);
}

static ssize_t store_enable_trace2sleep(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <=2) {
		enable_trace2sleep = input;
        t2w_switch = enable_trace2sleep;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE
static ssize_t show_gesture_min_x(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gesture_min_x);
}

static ssize_t store_gesture_min_x(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 5 && input <= 720) {
		gesture_min_x = input;
        smartwake_x_distance = gesture_min_x;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}


static ssize_t show_gesture_min_y(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gesture_min_y);
}

static ssize_t store_gesture_min_y(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 5 && input <= 1280) {
		gesture_min_y = input;
        smartwake_y_distance = gesture_min_y;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_gesture_vib(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gesture_vib);
}

static ssize_t store_gesture_vib(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		gesture_vib = input;
        smartwake_vib = gesture_vib;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE
static ssize_t show_smartwake_overrides_customgestures(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", smartwake_overrides_customgestures);
}

static ssize_t store_smartwake_overrides_customgestures(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input == 0) || (input == 1))) {
		smartwake_overrides_customgestures = input;
        if (input == 1)
            {
                wakeg_sysfs_all(0);
                smartwake_sysfs_toggle(1);
            }
        else
            {
                wakeg_sysfs_all(1);
                smartwake_sysfs_toggle(0);               
            }
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}
#endif

static ssize_t show_force_livedisplay(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", force_livedisplay);
}

static ssize_t store_force_livedisplay(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input >= 0) && (input <= 2))) {
		set_force_livedisplay(input);
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_daytime_r(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", daytime_r);
}

static ssize_t store_daytime_r(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>SCALE_VAL_MIN && input <=MAX_SCALE) {
		daytime_r = input;
        daytime_rgb_updated(daytime_r,daytime_g,daytime_b);
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_daytime_g(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", daytime_g);
}

static ssize_t store_daytime_g(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>SCALE_VAL_MIN && input <=MAX_SCALE) {
		daytime_g = input;
        daytime_rgb_updated(daytime_r,daytime_g,daytime_b);
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_daytime_b(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", daytime_b);
}

static ssize_t store_daytime_b(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>SCALE_VAL_MIN && input <=MAX_SCALE) {
		daytime_b = input;
        daytime_rgb_updated(daytime_r,daytime_g,daytime_b);
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_brightness_lvl(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", b_cache);
}

static ssize_t show_target_r(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", target_r);
}

static ssize_t store_target_r(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>SCALE_VAL_MIN && input <=MAX_SCALE) {
		target_r = input;
		target_achieved = 0;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_target_g(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", target_g);
}

static ssize_t store_target_g(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>SCALE_VAL_MIN && input <=MAX_SCALE) {
		target_g = input;
		target_achieved = 0;		
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_target_b(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", target_b);
}

static ssize_t store_target_b(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>SCALE_VAL_MIN && input <=MAX_SCALE) {
		target_b = input;
		target_achieved = 0;		
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_livedisplay_start_hour(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", livedisplay_start_hour);
}

static ssize_t store_livedisplay_start_hour(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>=0 && input < 24) {
		livedisplay_start_hour = input;
		calc_active_minutes();
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_livedisplay_stop_hour(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", livedisplay_stop_hour);
}

static ssize_t store_livedisplay_stop_hour(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>=0 && input < 24 && (input != livedisplay_start_hour)) {
		livedisplay_stop_hour = input;
		calc_active_minutes();
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_livedisplay_aggression(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", livedisplay_aggression);
}

static ssize_t store_livedisplay_aggression(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>0 && input < 10000) {
		livedisplay_aggression = input;
		target_achieved = 0;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t store_brightness_lvl(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>2 && input <=10) {
		b_cache = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_brightness_lvl_auto_enable(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", brightness_lvl_auto_enable);
}

static ssize_t store_brightness_lvl_auto_enable(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input == 0) || (input == 1))) {
		brightness_lvl_auto_enable = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_brightness_lvl_auto_hour_start(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", brightness_lvl_auto_hour_start);
}

static ssize_t store_brightness_lvl_auto_hour_start(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>=0 && input < 24 && (input != brightness_lvl_auto_hour_end)) {
		brightness_lvl_auto_hour_start = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_brightness_lvl_auto_hour_end(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", brightness_lvl_auto_hour_end);
}

static ssize_t store_brightness_lvl_auto_hour_end(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>=0 && input < 24 && (input != brightness_lvl_auto_hour_start)) {
		brightness_lvl_auto_hour_end = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_klapse_brightness_threshold(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", klapse_brightness_threshold);
}

static ssize_t store_klapse_brightness_threshold(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>0 && input<=255) {
		klapse_brightness_threshold = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_force_disable_gestures_HARD(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", force_disable_gestures_HARD);
}

static ssize_t store_force_disable_gestures_HARD(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input==0) || (input == 1))) {
		force_disable_gestures_HARD = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_force_enable_gestures_charging(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", force_enable_gestures_charging);
}

static ssize_t store_force_enable_gestures_charging(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input==0) || (input == 1))) {
		force_enable_gestures_charging = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_s2w_vibe(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", s2w_vibe);
}

static ssize_t store_s2w_vibe(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		s2w_vibe = input;
        s2w_vibration_level = s2w_vibe;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_gaming_mode(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", gaming_mode);
}

static ssize_t store_gaming_mode(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input==0) || (input == 1))) {
		gaming_mode = input;
		do_boost = input;
		set_override(do_boost);
		//if (!suspended) // I (as of now) see no reason why cpu should be updated only when not suspended.
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

/*************************************************************************************************/

#define define_global_rw_attr(_name)		\
static struct global_attr _name##_attr =	\
	__ATTR(_name, 0644, show_##_name, store_##_name)

define_global_rw_attr(debug_mask);
define_global_rw_attr(up_rate_us);
define_global_rw_attr(down_rate_us);
define_global_rw_attr(sleep_ideal_freq);
define_global_rw_attr(sleep_wakeup_freq);
define_global_rw_attr(awake_ideal_freq);
define_global_rw_attr(sample_rate_jiffies);
define_global_rw_attr(ramp_up_step);
define_global_rw_attr(ramp_down_step);
define_global_rw_attr(max_cpu_load);
define_global_rw_attr(min_cpu_load);
define_global_rw_attr(dt2w_vib);
define_global_rw_attr(dt2w_on_buttons_enabled);
define_global_rw_attr(WificustPowerMode);
define_global_rw_attr(x_left);
define_global_rw_attr(x_right);
define_global_rw_attr(y_up);
define_global_rw_attr(y_down);
define_global_rw_attr(pocket_mod_enabled);
define_global_rw_attr(revert_dt2w_area_bounded_by_xy);
define_global_rw_attr(ac_charge_level);
define_global_rw_attr(usb_charge_level);
define_global_rw_attr(dt2w_time_for_tap);
define_global_rw_attr(dt2w_tap_distance);
define_global_rw_attr(vibrator_on_lock);
define_global_rw_attr(vib_power_in_pocket_mod_dt2w_override);
define_global_rw_attr(pocket_mode_dt2w_override_taps_timeout);
define_global_rw_attr(pocket_mode_dt2w_override_taps_number);
define_global_rw_attr(enable_trace2sleep);

//Livedisplay ones
define_global_rw_attr(force_livedisplay);
define_global_rw_attr(daytime_r);
define_global_rw_attr(daytime_g);
define_global_rw_attr(daytime_b);
define_global_rw_attr(brightness_lvl);
define_global_rw_attr(target_r);
define_global_rw_attr(target_g);
define_global_rw_attr(target_b);
define_global_rw_attr(livedisplay_start_hour);
define_global_rw_attr(livedisplay_stop_hour);
define_global_rw_attr(livedisplay_aggression);
define_global_rw_attr(brightness_lvl_auto_hour_start);
define_global_rw_attr(brightness_lvl_auto_hour_end);
define_global_rw_attr(brightness_lvl_auto_enable);
define_global_rw_attr(klapse_brightness_threshold);

#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE
define_global_rw_attr(gesture_min_y);
define_global_rw_attr(gesture_min_x);
define_global_rw_attr(gesture_vib);
define_global_rw_attr(smartwake_overrides_customgestures);
#endif

//define_global_rw_attr(min_swipe_radius);
define_global_rw_attr(s2w_vibe);
define_global_rw_attr(force_disable_gestures_HARD);
define_global_rw_attr(force_enable_gestures_charging);
define_global_rw_attr(gaming_mode);


static struct attribute * thunderx_attributes[] = {
	&debug_mask_attr.attr,
	&up_rate_us_attr.attr,
	&down_rate_us_attr.attr,
	&sleep_ideal_freq_attr.attr,
	&sleep_wakeup_freq_attr.attr,
	&awake_ideal_freq_attr.attr,
	&sample_rate_jiffies_attr.attr,
	&ramp_up_step_attr.attr,
	&ramp_down_step_attr.attr,
	&max_cpu_load_attr.attr,
	&min_cpu_load_attr.attr,
    &dt2w_vib_attr.attr,
    &dt2w_on_buttons_enabled_attr.attr,
    &WificustPowerMode_attr.attr,
    &x_left_attr.attr,
    &x_right_attr.attr,
    &y_up_attr.attr,
    &y_down_attr.attr,
    &pocket_mod_enabled_attr.attr,
    &revert_dt2w_area_bounded_by_xy_attr.attr,
	&ac_charge_level_attr.attr,
	&usb_charge_level_attr.attr,
    &dt2w_tap_distance_attr.attr,
    &dt2w_time_for_tap_attr.attr,
    &vibrator_on_lock_attr.attr,
    &vib_power_in_pocket_mod_dt2w_override_attr.attr,
    &pocket_mode_dt2w_override_taps_timeout_attr.attr,
    &pocket_mode_dt2w_override_taps_number_attr.attr,
    &enable_trace2sleep_attr.attr,
#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE    
	&gesture_min_x_attr.attr,
    &gesture_min_y_attr.attr,
    &gesture_vib_attr.attr,
    &smartwake_overrides_customgestures_attr.attr,
#endif    
//    &min_swipe_radius_attr.attr,
    &s2w_vibe_attr.attr,
// Livedisplay ones
    &force_livedisplay_attr.attr,
    &daytime_r_attr.attr,
    &daytime_g_attr.attr,
    &daytime_b_attr.attr,
    &brightness_lvl_attr.attr,
    &target_r_attr.attr,
    &target_g_attr.attr,
    &target_b_attr.attr,
    &livedisplay_start_hour_attr.attr,
    &livedisplay_stop_hour_attr.attr,
    &livedisplay_aggression_attr.attr,
    &brightness_lvl_auto_hour_start_attr.attr,    
    &brightness_lvl_auto_hour_end_attr.attr,  
    &brightness_lvl_auto_enable_attr.attr,
    &klapse_brightness_threshold_attr.attr,
    &force_disable_gestures_HARD_attr.attr, 
    &force_enable_gestures_charging_attr.attr,
	&gaming_mode_attr.attr,
	NULL,
};

static struct attribute_group thunderx_attr_group = {
	.attrs = thunderx_attributes,
	.name = "Poison",
};

static int cpufreq_governor_thunderx(struct cpufreq_policy *new_policy,
		unsigned int event)
{
	unsigned int cpu = new_policy->cpu;
	int rc;
	struct thunderx_info_s *this_thunderx = &per_cpu(thunderx_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!new_policy->cur))
			return -EINVAL;

		this_thunderx->cur_policy = new_policy;

		this_thunderx->enable = 1;

		thunderx_update_min_max(this_thunderx,new_policy,suspended);

		this_thunderx->freq_table = cpufreq_frequency_get_table(cpu);
		pr_info("thunderX: starting thunderx algorithm\n");
		if (!this_thunderx->freq_table)
			printk(KERN_WARNING "thunderX: no frequency table for cpu %d?!\n",cpu);

		smp_wmb();

		// Do not register the idle hook and create sysfs
		// entries if we have already done so.
		if (atomic_inc_return(&active_count) <= 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&thunderx_attr_group);
			if (rc)
				return rc;
		}

		if (!timer_pending(&this_thunderx->timer))
			reset_timer(cpu,this_thunderx);

		cpufreq_thunderx_timer(cpu);

		break;

	case CPUFREQ_GOV_LIMITS:
		thunderx_update_min_max(this_thunderx,new_policy,suspended);

		if (this_thunderx->cur_policy->cur > new_policy->max) {
			dprintk(THUNDERX_DEBUG_JUMPS,"thunderX: jumping to new max freq: %d\n",new_policy->max);
			__cpufreq_driver_target(this_thunderx->cur_policy,
						new_policy->max, CPUFREQ_RELATION_H);
		}
		else if (this_thunderx->cur_policy->cur < new_policy->min) {
			dprintk(THUNDERX_DEBUG_JUMPS,"thunderX: jumping to new min freq: %d\n",new_policy->min);
			__cpufreq_driver_target(this_thunderx->cur_policy,
						new_policy->min, CPUFREQ_RELATION_L);
		}

		if (this_thunderx->cur_policy->cur < new_policy->max && !timer_pending(&this_thunderx->timer))
			reset_timer(cpu,this_thunderx);

		break;

	case CPUFREQ_GOV_STOP:
		this_thunderx->enable = 0;
		smp_wmb();
		del_timer(&this_thunderx->timer);
		flush_work(&freq_scale_work);
		this_thunderx->idle_exit_time = 0;

		break;
	}

	return 0;
}

static void thunderx_suspend(int cpu, int suspend)
{
	struct thunderx_info_s *this_thunderx = &per_cpu(thunderx_info, cpu);
	struct cpufreq_policy *policy = this_thunderx->cur_policy;
	unsigned int new_freq;

	if (!this_thunderx->enable)
		return;

	thunderx_update_min_max(this_thunderx,policy,suspend);
	if (!suspend) { // resume at max speed:
		new_freq = validate_freq(policy,sleep_wakeup_freq);       

		dprintk(THUNDERX_DEBUG_JUMPS,"thunderX: awaking at %d\n",new_freq);

		__cpufreq_driver_target(policy, new_freq,
					CPUFREQ_RELATION_L);
	} else {
		// to avoid wakeup issues with quick sleep/wakeup don't change actual frequency when entering sleep
		// to allow some time to settle down. Instead we just reset our statistics (and reset the timer).
		// Eventually, the timer will adjust the frequency if necessary.

		this_thunderx->freq_change_time_in_idle =
			get_cpu_idle_time_us(cpu,&this_thunderx->freq_change_time);

		dprintk(THUNDERX_DEBUG_JUMPS,"thunderX: suspending at %d\n",policy->cur);
	}

	reset_timer(cpu,this_thunderx);
}

static void thunderx_early_suspend(struct early_suspend *handler) {
	int i;
	if (suspended || sleep_ideal_freq==0) // disable behavior for sleep_ideal_freq==0
		return;
	suspended = 1;
	for_each_online_cpu(i)
		thunderx_suspend(i,1);
}

static void thunderx_late_resume(struct early_suspend *handler) {
	int i;
	if (!suspended) // already not suspended so nothing to do
		return;
	suspended = 0;
	for_each_online_cpu(i)
		thunderx_suspend(i,0);
}

static struct early_suspend thunderx_power_suspend = {
	.suspend = thunderx_early_suspend,
	.resume = thunderx_late_resume,
#ifdef CONFIG_MACH_HERO
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
#endif
};

static int __init cpufreq_thunderx_init(void)
{
	unsigned int i;
	struct thunderx_info_s *this_thunderx;
	debug_mask = 0;
	up_rate_us = DEFAULT_UP_RATE_US;
	down_rate_us = DEFAULT_DOWN_RATE_US;
	sleep_ideal_freq = DEFAULT_SLEEP_IDEAL_FREQ;
	sleep_wakeup_freq = DEFAULT_SLEEP_WAKEUP_FREQ;
	awake_ideal_freq = DEFAULT_AWAKE_IDEAL_FREQ;
	sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;
	ramp_up_step = DEFAULT_RAMP_UP_STEP;
	ramp_down_step = DEFAULT_RAMP_DOWN_STEP;
	max_cpu_load = DEFAULT_MAX_CPU_LOAD;
	min_cpu_load = DEFAULT_MIN_CPU_LOAD;
    

// ********* Venom kernel additions for A7000 (aio_row) inits **********

    dt2w_vib = 50;
    WificustPowerMode = custPowerMode;
    x_left = 0;
    x_right = 720;
    y_up = 0;
    y_down = 1280;
    left = 0;
    right = 720;
    co_up = 0;
    co_down = 1280;
    pocket_mod_enabled=1;
    revert_dt2w_area_bounded_by_xy=0;
	ac_charge_level = AC_CHARGE_LEVEL_DEFAULT;
	usb_charge_level = USB_CHARGE_LEVEL_DEFAULT;
    dt2w_tap_distance=80;
    dt2w_time_for_tap=350;
    vibrator_on_lock = false;
    vib_power_in_pocket_mod_dt2w_override = 150;
    pocket_mode_dt2w_override_taps_timeout = 550;
    pocket_mode_dt2w_override_taps_number = 3;
    enable_trace2sleep = 1;         
    
#ifdef CONFIG_TOUCHSCREEN_SMARTWAKE       
	gesture_min_x = 400;
    gesture_min_y = 400;
    gesture_vib = 50;
    smartwake_overrides_customgestures = 1;
#endif    

    //min_swipe_radius = 400;
    s2w_vibe = 50;
    force_disable_gestures_HARD = false;
    

// ******** End of inits of venom kernel *******

	spin_lock_init(&cpumask_lock);

	suspended = 0;

	/* Initalize per-cpu data: */
	for_each_possible_cpu(i) {
		this_thunderx = &per_cpu(thunderx_info, i);
		this_thunderx->enable = 0;
		this_thunderx->cur_policy = 0;
		this_thunderx->ramp_dir = 0;
		this_thunderx->time_in_idle = 0;
		this_thunderx->idle_exit_time = 0;
		this_thunderx->freq_change_time = 0;
		this_thunderx->freq_change_time_in_idle = 0;
		this_thunderx->cur_cpu_load = 0;
		// intialize timer:
		init_timer_deferrable(&this_thunderx->timer);
		this_thunderx->timer.function = cpufreq_thunderx_timer;
		this_thunderx->timer.data = i;
		work_cpumask_test_and_clear(i);
	}

	// Scale up is high priority
	up_wq = alloc_workqueue("thunderx_up", WQ_HIGHPRI, 1);
	down_wq = alloc_workqueue("thunderx_down", 0, 1);
	if (!up_wq || !down_wq)
		return -ENOMEM;

	INIT_WORK(&freq_scale_work, cpufreq_thunderx_freq_change_time_work);

	register_early_suspend(&thunderx_power_suspend);

	return cpufreq_register_governor(&cpufreq_gov_poison);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_POISON
fs_initcall(cpufreq_thunderx_init);
#else
module_init(cpufreq_thunderx_init);
#endif

static void __exit cpufreq_thunderx_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_poison);
	destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_thunderx_exit);


MODULE_AUTHOR ("tanish2k09 <tanish2k09.dev@gmail.com");
MODULE_DESCRIPTION ("'cpufreq_poison' - A smart cpufreq governor based on thunderx");
MODULE_LICENSE ("GPL");
