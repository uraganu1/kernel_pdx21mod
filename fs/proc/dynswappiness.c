/*
 * fs/proc/dynswappiness.c
 *
 *
 * Copyright (c) 2020, Andrei Cojocar <cojocar.andrei@gmail.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/swap.h>
#include <linux/tweaks.h>

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "Andrei Cojocar <cojocar.andrei@gmail.com>"
#define DRIVER_DESCRIPTION "Dynamic swappiness"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[dynswappiness]: "

#define K(x) ((x) << (PAGE_SHIFT - 10))
#define DEFAULT_DS_STATE 	1
#define MAXIMUM_SWAPPINESS 	100
#define MINIMUM_SWAPPINESS 	0
#define CHK_CRITICAL_TIME	8000
#define CHK_RESTRICTED_TIME 	16000
#define CHK_NORMAL_TIME		32000
#define NO_CHANGES_COUNTER_MAX	128
#define NO_CHANGES_CRITICAL_MAX 8

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

static struct delayed_work check_mem_work;
static atomic_t ds_state;
static bool init_chk = true;
static struct mutex ds_lock;
static unsigned int sfp_level = 0;
static unsigned int no_changes_counter = 0;
static unsigned int check_time = CHK_CRITICAL_TIME;
static atomic_t check_time_mul;
static struct notifier_block lcd_notif;

extern int lcd_register_client(struct notifier_block *nb);
extern int lcd_unregister_client(struct notifier_block *nb);

static int get_new_swappiness(int level)
{
        if (level < 0)
                return MINIMUM_SWAPPINESS;
        if (level > MAXIMUM_SWAPPINESS)
                return MAXIMUM_SWAPPINESS;
        return level;
}

static void ds_set_swappiness(unsigned int c_swappiness, int n_diff)
{
	int level;

	if (((c_swappiness >= MAXIMUM_SWAPPINESS) && (n_diff >= 0)) ||
	   ((c_swappiness <= MINIMUM_SWAPPINESS) && (n_diff <= 0)))
		return;

	level = get_new_swappiness(c_swappiness + n_diff);

	vm_swappiness = level;
}

static unsigned int ds_get_swappiness(void)
{
	int val;

	val = vm_swappiness;

	if (val <= MINIMUM_SWAPPINESS)
		return MINIMUM_SWAPPINESS;
	if (val >= MAXIMUM_SWAPPINESS)
		return MAXIMUM_SWAPPINESS;
	return val;
}

static int lcd_notifier_cb(struct notifier_block *nb,
                           unsigned long event,
                           void *data)
{
       if ((event == LCD_EVENT_ON) || (event == LCD_EVENT_DOZE_SUSPEND)) {
	       atomic_set(&check_time_mul, 1);
               pr_debug("dbg dynswappiness -> LCD EVENT ON\n");
       }
       else {
	       atomic_set(&check_time_mul, 8);
	       pr_debug("dbg dynswappiness -> LCD EVENT OFF\n");
       }
       return NOTIFY_OK;
}

static void check_mem_func(struct work_struct *work)
{
	struct sysinfo i;
	unsigned int swap_total = 0;
	unsigned int swap_free = 0;
	unsigned int critical_level, restriction_level, normal_level, current_swappiness;
	int swap_free_diff;

	mutex_lock(&ds_lock);
	si_swapinfo(&i);
	swap_total = K(i.totalswap);
	swap_free = K(i.freeswap);

	if (swap_total > 0) {
		if (init_chk) {
			init_chk = false;
		}
		else {
			critical_level = swap_total/16;
			restriction_level = swap_total/8;
			normal_level = swap_total/4;
			swap_free_diff = sfp_level - swap_free;
			current_swappiness = ds_get_swappiness();
			no_changes_counter++;

			if (swap_free >= normal_level) {
				if (current_swappiness != MAXIMUM_SWAPPINESS)
					ds_set_swappiness(MAXIMUM_SWAPPINESS-1, 1);
				check_time = CHK_NORMAL_TIME;
				no_changes_counter = 0;
			}
			else if (swap_free <= critical_level) {
				if (swap_free_diff == 0) {
                                        if (no_changes_counter > NO_CHANGES_CRITICAL_MAX)
                                                ds_set_swappiness(current_swappiness, -4);
                                } else if (swap_free_diff > 0) {
                			if (swap_free_diff > swap_total/32) {
						ds_set_swappiness(current_swappiness, -16);
					}
					else if (swap_free_diff > swap_total/64) {
						ds_set_swappiness(current_swappiness, -8);
					}
					else {
						ds_set_swappiness(current_swappiness, -4);
					}
				}
				check_time = CHK_CRITICAL_TIME;
			}
			else if (swap_free < restriction_level) {
				if (swap_free_diff >= 0) {
                                        if (swap_free_diff > swap_total/16) {
                                                ds_set_swappiness(current_swappiness, -4);
                                        }
                                        else if (swap_free_diff > swap_total/32) {
                                                ds_set_swappiness(current_swappiness, -2);
                                        }
                                        else if (swap_free_diff > swap_total/64) {
                                                ds_set_swappiness(current_swappiness, -1);
                                        }
                                }
                                else {
					swap_free_diff = swap_free - sfp_level;
                                        if (swap_free_diff > swap_total/16) {
                                                ds_set_swappiness(current_swappiness, +4);
                                        }
                                        else if (swap_free_diff > swap_total/32) {
                                                ds_set_swappiness(current_swappiness, +2);
                                        }
                                        else if (swap_free_diff > swap_total/64) {
                                                ds_set_swappiness(current_swappiness, +1);
                                        }
                                }
				if (no_changes_counter > NO_CHANGES_COUNTER_MAX)
					ds_set_swappiness(current_swappiness, +1);
                                check_time = CHK_RESTRICTED_TIME;
			}
			else {
				if (swap_free_diff >= 0) {
					if (swap_free_diff > swap_total/16) {
                                                ds_set_swappiness(current_swappiness, -2);
                                        }
                                        else if (swap_free_diff > swap_total/32) {
                                                ds_set_swappiness(current_swappiness, -1);
                                        }
				}
				else {
					swap_free_diff = swap_free - sfp_level;
					if (swap_free_diff > swap_total/16) {
                                                ds_set_swappiness(current_swappiness, +2);
                                        }
                                        else if (swap_free_diff > swap_total/32) {
                                                ds_set_swappiness(current_swappiness, +1);
                                        }
				}
				if (no_changes_counter > NO_CHANGES_COUNTER_MAX)
					ds_set_swappiness(current_swappiness, +2);
				check_time = CHK_NORMAL_TIME;
			}
		}
		sfp_level = swap_free;
		queue_delayed_work(system_power_efficient_wq,
                                   &check_mem_work, msecs_to_jiffies(check_time * atomic_read(&check_time_mul)));
	}
	mutex_unlock(&ds_lock);
}

void ds_set_state(int state)
{
	int val = atomic_read(&ds_state);
	if (val != state) {
		cancel_delayed_work_sync(&check_mem_work);
		if (state == 0) {
			pr_debug("dbg dynswappiness - stop dynamic swappiness\n");
			ds_set_swappiness(MAXIMUM_SWAPPINESS-1, 1);
			init_chk = true;
		}
		else {
			pr_debug("dbg dynswappiness - start dynamic swappiness\n");
			queue_delayed_work(system_power_efficient_wq,
                                           &check_mem_work, CHK_CRITICAL_TIME);
		}
		atomic_set(&ds_state, state);
	}
}

EXPORT_SYMBOL(ds_set_state);

static int __init dynswappiness_init(void)
{
	int ret = 0;

	atomic_set(&ds_state, DEFAULT_DS_STATE);
	atomic_set(&check_time_mul, 1);
	init_chk = true;

	mutex_init(&ds_lock);
	INIT_DELAYED_WORK(&check_mem_work, check_mem_func);
	lcd_notif.notifier_call = lcd_notifier_cb;
	ret = lcd_register_client(&lcd_notif);
	if (ret) {
		pr_err("Failed to register lcd notifier, err: %d\n", ret);
	}
	if (1 == atomic_read(&ds_state)) {
		queue_delayed_work(system_power_efficient_wq,
                                   &check_mem_work, msecs_to_jiffies(CHK_NORMAL_TIME*8));
		ds_set_state(DEFAULT_DS_STATE);
	}
	return 0;
}

static void __exit dynswappiness_exit(void)
{
	lcd_unregister_client(&lcd_notif);
	cancel_delayed_work_sync(&check_mem_work);
	mutex_destroy(&ds_lock);
}

module_init(dynswappiness_init);
module_exit(dynswappiness_exit);

