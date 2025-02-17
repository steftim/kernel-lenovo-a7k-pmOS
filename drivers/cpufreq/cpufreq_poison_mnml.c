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
#include "../drivers/misc/mediatek/connectivity/conn_soc/drv_wlan/mt_wifi/wlan/include/wlan_pms.h"
#include <linux/pocket_mod.h>
#include "../drivers/power/mediatek/fastchg.h"

// Livedisplay headers
#include <uapi/linux/time.h>
#include <uapi/linux/rtc.h>
#include <linux/rtc.h>

/******************** Tunable parameters: ********************/

/*
 * The "ideal" frequency to use when awake. The governor will ramp up faster
 * towards the ideal frequency and slower after it has passed it. Similarly,
 * lowering the frequency towards the ideal frequency is faster than below it.
 */
#define DEFAULT_AWAKE_IDEAL_FREQ 1287000
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
#define DEFAULT_RAMP_UP_STEP 30000
static unsigned int ramp_up_step;

/*
 * Frequency delta when ramping down below the ideal freqeuncy.
 * Zero disables and will calculate ramp down according to load heuristic.
 * When above the ideal freqeuncy we always ramp down to the ideal freq.
 */
#define DEFAULT_RAMP_DOWN_STEP 100000
static unsigned int ramp_down_step;

/*
 * CPU freq will be increased if measured load > max_cpu_load;
 */
#define DEFAULT_MAX_CPU_LOAD 60
static unsigned long max_cpu_load;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
#define DEFAULT_MIN_CPU_LOAD 45
static unsigned long min_cpu_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp up.
 * Notice we ignore this when we are below the ideal frequency.
 */
#define DEFAULT_UP_RATE_US 100000;
static unsigned long up_rate_us;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 * Notice we ignore this when we are above the ideal frequency.
 */
#define DEFAULT_DOWN_RATE_US 50000;
static unsigned long down_rate_us;

/*
 * The frequency to set when waking up from sleep.
 * When sleep_ideal_freq=0 this will have no effect.
 */
#define DEFAULT_SLEEP_WAKEUP_FREQ 1495000
static unsigned int sleep_wakeup_freq;

/*
 * Sampling rate, highly recommended to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

/*
 ******* Custom tunables for A7000 (aio_row) ********
 */

static bool pocket_mod_enabled;
static int WificustPowerMode, ac_charge_level, usb_charge_level;

// Kernel - only livedisplay variables
static bool force_livedisplay, target_achieved;
static struct timeval time;
static unsigned long local_time;
static struct rtc_time tm;
extern void force_livedisplay_set_rgb(int force_r, int force_g, int force_b);
static int daytime_r, daytime_g, daytime_b, target_r, target_g, target_b, current_r, current_g, current_b;
static unsigned int brightness_lvl;
static unsigned int active_minutes, livedisplay_aggression;
static int livedisplay_start_hour, livedisplay_stop_hour;
//////////////////////////////////////

bool no_vibrate=false;
bool vibrator_on_lock = true;

// Mode for making specific set of frequencies available to gov
// 1 = normal
// 2 = performance
// 0 = battery
// 3 = min-max
// 4 = mean (use only middle frequency)
#define DEFAULT_MODE 1
static unsigned int mode;

// Defining various frequencies to mean about in different modes
// Mode 1 has no specific set. All freq are usable.
// Mode 2 should use only upper 3 available freq.
// Mode 0 should use only lower 3 freq.
// Mode 3 should use only max and min freq... Just like max-min CPU governor. No need to define default ofc.
// Mode 4 is fixed at the middle freq.
#define DEFAULT_MODE_LOW_FREQ 936000
static unsigned int mode_low_freq;

#define DEFAULT_MODE_HIGH_FREQ 1287000
static unsigned int mode_high_freq;

#define DEFAULT_MODE_MID_FREQ 1170000
static unsigned int mode_mid_freq;


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

//Livedisplay related functions
static void calc_active_minutes()
{
    active_minutes = (24 + livedisplay_stop_hour - livedisplay_start_hour)*60;
}

static int get_minutes_since_start()
{
    int hour = tm.tm_hour - livedisplay_start_hour;
    
    if (hour < 0)
        hour += 24;
        
    int min = ((hour*60) + tm.tm_min);
    return min;
}

static int get_minutes_before_stop()
{
    int min = (active_minutes - get_minutes_since_start());
    return min;
}

static void force_livedisplay_set_rgb_brightness(int r,int g,int b)
{
    r = (int)((r*brightness_lvl)/10);
    g = (int)((g*brightness_lvl)/10);
    b = (int)((b*brightness_lvl)/10);
    
    if (r < 0)
        r = 50;
    else if (r > daytime_r)
        r = daytime_r;
    if (g < 0)
        g = 50;
    else if (g > daytime_g)
        g = daytime_g;
    if (b < 0)
        b = 50;
    else if (b > daytime_b)
        b = daytime_b;
    
    force_livedisplay_set_rgb(r,g,b);
}

//Livedisplay functions end here.


inline static void thunderx_update_min_max(struct thunderx_info_s *this_thunderx, struct cpufreq_policy *policy, int suspend) {
	if (suspend) {
        no_vibrate=true;
		this_thunderx->ideal_speed = // sleep_ideal_freq; but make sure it obeys the policy min/max
			policy->max > sleep_ideal_freq ?
			(sleep_ideal_freq > policy->min ? sleep_ideal_freq : policy->min) : policy->max;
	} else {
        no_vibrate=false;
		this_thunderx->ideal_speed = // awake_ideal_freq; but make sure it obeys the policy min/max
			policy->min < awake_ideal_freq ?
			(awake_ideal_freq < policy->max ? awake_ideal_freq : policy->max) : policy->min;
	}

            // LiveDisplay function masking
            if (force_livedisplay == 1)
            {
                // Get time
              	do_gettimeofday(&time);
	            local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	            rtc_time_to_tm(local_time, &tm);
	            
	            if(!((tm.tm_hour >= livedisplay_start_hour) || (tm.tm_hour < livedisplay_stop_hour))) //Means not in livedisplay time period.
	            {
	                force_livedisplay_set_rgb_brightness(daytime_r,daytime_g,daytime_b); 
	                target_achieved = 0;
	            }
                
                else if (target_achieved == 0)
                {
                    current_r = daytime_r - (((daytime_r - target_r)*(get_minutes_since_start())*livedisplay_aggression)/(active_minutes*10));
                    current_g = daytime_g - (((daytime_g - target_g)*(get_minutes_since_start())*livedisplay_aggression)/(active_minutes*10));
                    current_b = daytime_b - (((daytime_b - target_b)*(get_minutes_since_start())*livedisplay_aggression)/(active_minutes*10));
                    
                    if ((current_r <= target_r) && (current_g <= target_g) && (current_b <= target_b))
                    {
                        target_achieved = 1;
                        current_r = target_r;
                        current_g = target_g;
                        current_b = target_b;
                    }    
                    
                    force_livedisplay_set_rgb_brightness(current_r, current_g, current_b);                                                  
               
                }
                else if (target_achieved == 1)
                {
                    if(get_minutes_before_stop() > 10)
                    {
                        force_livedisplay_set_rgb_brightness(target_r, target_g, target_b);
                    }
                    else if(get_minutes_before_stop() <= 10)
                    {                    
                        current_r = target_r + (((daytime_r - target_r)*(10 - get_minutes_before_stop()))/10);
                        current_g = target_g + (((daytime_g - target_g)*(10 - get_minutes_before_stop()))/10);
                        current_b = target_b + (((daytime_b - target_b)*(10 - get_minutes_before_stop()))/10);
                        
                        force_livedisplay_set_rgb_brightness(current_r, current_g, current_b);
                    }                                        
                }                
            }
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

    if (mode==4)                // This is to check for mid-only mode.
        return mode_mid_freq;     // Return mid freq in mid-only mode.
    if ((mode==0)&&(freq>mode_low_freq))     // checking for battery mode and frequency level
    {
        awake_ideal_freq = 936000;
        return mode_low_freq;     // return max cap of battery frequency if requested freq is higher.
    }
    if ((mode==2)&&(freq<mode_high_freq)) // check performance mode
    {
        awake_ideal_freq = (1417000<policy->max) ? 1417000 : policy->max;

        return mode_high_freq;         // return least cap of performance if requested is lesser
    }
    if (mode==3) //min-max
    {
        if ((freq/936000)>1)
            return  policy->max;
        return policy->min;
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


static ssize_t show_mode(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", mode);
}

static ssize_t store_mode(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input<=4) {
		mode = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}


static ssize_t show_mode_low_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", mode_low_freq);
}

static ssize_t store_mode_low_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 468000 && input<=1495000) {
		mode_low_freq = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_mode_high_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", mode_high_freq);
}

static ssize_t store_mode_high_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 468000 && input<=1495000) {
		mode_high_freq = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_mode_mid_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", mode_mid_freq);
}

static ssize_t store_mode_mid_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 468000 && input<=1495000) {
		mode_mid_freq = input;
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

static ssize_t show_force_livedisplay(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", force_livedisplay);
}

static ssize_t store_force_livedisplay(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && ((input == 0) || (input == 1))) {
		force_livedisplay = input;
        if (force_livedisplay == 0)
        {
            force_livedisplay_set_rgb_brightness(daytime_r, daytime_g, daytime_b);
            target_achieved = 0;
        }
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
	if (res >= 0 && input>50 && input <=2000) {
		daytime_r = input;
		if ((force_livedisplay == 0) || ((force_livedisplay == 1) && (tm.tm_hour < 18) && (tm.tm_hour >=7))) 
		    force_livedisplay_set_rgb_brightness(daytime_r, daytime_g, daytime_b);
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
	if (res >= 0 && input>50 && input <=2000) {
		daytime_g = input;
		if ((force_livedisplay == 0) || ((force_livedisplay == 1) && (tm.tm_hour < 18) && (tm.tm_hour >=7))) 
		    force_livedisplay_set_rgb_brightness(daytime_r, daytime_g, daytime_b);
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
	if (res >= 0 && input>50 && input <=2000) {
		daytime_b = input;
		if ((force_livedisplay == 0) || ((force_livedisplay == 1) && (tm.tm_hour < 18) && (tm.tm_hour >=7))) 
		    force_livedisplay_set_rgb_brightness(daytime_r, daytime_g, daytime_b);
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
}

static ssize_t show_brightness_lvl(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", brightness_lvl);
}

static ssize_t store_brightness_lvl(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input>1 && input <=10) {
		brightness_lvl = input;
		if (!suspended)
			thunderx_update_min_max_allcpus();
	}
	else return -EINVAL;
	return count;
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
	if (res >= 0 && input>50 && input <=2000) {
		target_r = input;
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
	if (res >= 0 && input>50 && input <=2000) {
		target_g = input;
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
	if (res >= 0 && input>50 && input <=2000) {
		target_b = input;
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
		if (!suspended)
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
define_global_rw_attr(mode);
define_global_rw_attr(mode_low_freq);
define_global_rw_attr(mode_high_freq);
define_global_rw_attr(mode_mid_freq);
define_global_rw_attr(WificustPowerMode);
define_global_rw_attr(pocket_mod_enabled);
define_global_rw_attr(ac_charge_level);
define_global_rw_attr(usb_charge_level);
define_global_rw_attr(vibrator_on_lock);
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
    &mode_attr.attr,
    &mode_low_freq_attr.attr,
    &mode_high_freq_attr.attr,
    &mode_mid_freq_attr.attr,
    &WificustPowerMode_attr.attr,
    &pocket_mod_enabled_attr.attr,
	&ac_charge_level_attr.attr,
	&usb_charge_level_attr.attr,
    &vibrator_on_lock_attr.attr,
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

    mode = DEFAULT_MODE;
    mode_low_freq = DEFAULT_MODE_LOW_FREQ;
    mode_high_freq = DEFAULT_MODE_HIGH_FREQ;
    mode_mid_freq = DEFAULT_MODE_MID_FREQ;
    
    
    WificustPowerMode = custPowerMode;
	ac_charge_level = AC_CHARGE_LEVEL_DEFAULT;
	usb_charge_level = USB_CHARGE_LEVEL_DEFAULT;
    vibrator_on_lock = true;

    force_livedisplay = 0;	
    
	do_gettimeofday(&time);
	local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &tm);

    daytime_r = 2000;
    daytime_g = 2000;
    daytime_b = 2000;
    brightness_lvl = 10;
    target_r = 2000;
    target_g = 1370;
    target_b = 700;
    livedisplay_aggression = 33;
    livedisplay_start_hour = 18;
    livedisplay_stop_hour = 7;
    active_minutes = (13*60);
    target_achieved = 0;
    

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
