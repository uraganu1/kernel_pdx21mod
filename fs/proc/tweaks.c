/*
 * fs/proc/tweaks.c
 *
 *
 * Copyright (c) 2024, Andrei Cojocar <cojocar.andrei@gmail.com>
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
#define DRIVER_DESCRIPTION "Android tweaks interface"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[tweaks]: "

#define DEFAULT_DS_STATE 1
#define DEFAULT_ZRAM_OVERWRITE 1
#define DEFAULT_CPUFREQCTRL 0

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

struct kobject *tweaks_kfpobj = NULL;

static BLOCKING_NOTIFIER_HEAD(tweaks_notifier_list);
static BLOCKING_NOTIFIER_HEAD(lcd_notifier_list);

static atomic_t ds_val;
static atomic_t zram_val;
static atomic_t cpufreqctrl_val;

int tweaks_register_client(struct notifier_block *nb)
{
       return blocking_notifier_chain_register(&tweaks_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(tweaks_register_client);

int tweaks_unregister_client(struct notifier_block *nb)
{
       return blocking_notifier_chain_unregister(&tweaks_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(tweaks_unregister_client);

int tweaks_notifier_call_chain(unsigned long val, void *data)
{
       return blocking_notifier_call_chain(&tweaks_notifier_list, val, &data);
}
EXPORT_SYMBOL_GPL(tweaks_notifier_call_chain);

int lcd_register_client(struct notifier_block *nb)
{
       return blocking_notifier_chain_register(&lcd_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(lcd_register_client);

int lcd_unregister_client(struct notifier_block *nb)
{
       return blocking_notifier_chain_unregister(&lcd_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(lcd_unregister_client);

int lcd_notifier_call_chain(unsigned long val, void *data)
{
       return blocking_notifier_call_chain(&lcd_notifier_list, val, &data);
}
EXPORT_SYMBOL_GPL(lcd_notifier_call_chain);

unsigned int is_zramsize_overwritten(void)
{
       return (unsigned int)atomic_read(&zram_val);
}
EXPORT_SYMBOL_GPL(is_zramsize_overwritten);

extern void ds_set_state(int state);

static void tweaks_ds_set_state(int state)
{
	atomic_set(&ds_val, state);
	ds_set_state(state);
}

static ssize_t fs_dynswappiness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;

        count += sprintf(buf, "%d\n", (int)atomic_read(&ds_val));
        return count;
}

static ssize_t fs_dynswappiness_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int r, val;

        r = kstrtoint(buf, 10, &val);
        if ((r) || (val < 0)) {
                return -EINVAL;
        }
        if (val > 1) {
                val = 1;
        }

        tweaks_ds_set_state(val);

        return count;
}

static DEVICE_ATTR(dynswappiness, 0644, fs_dynswappiness_show, fs_dynswappiness_dump);

static ssize_t zram_overwrite_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        size_t count = 0;

        count += sprintf(buf, "%d\n", (int)atomic_read(&zram_val));
        return count;
}

static ssize_t zram_overwrite_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int r, val;

        r = kstrtoint(buf, 10, &val);
        if ((r) || (val < 0)) {
                return -EINVAL;
        }
        if (val > 1) {
                val = 1;
        }

        atomic_set(&zram_val, val);

        return count;
}

static DEVICE_ATTR(zramoverwrite, 0644, zram_overwrite_show, zram_overwrite_dump);

static void tweaks_cpufreqctrlena_set_val(int val)
{
        atomic_set(&cpufreqctrl_val, val);
        tweaks_notifier_call_chain(TWEAKS_CPUFQCTRL_EVENT, (void *)&val);
}

static ssize_t cpufreqctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        size_t count = 0;

        count += sprintf(buf, "%d\n", (int)atomic_read(&cpufreqctrl_val));
        return count;
}

static ssize_t cpufreqctrl_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int r, val;

        r = kstrtoint(buf, 10, &val);
        if ((r) || (val < 0)) {
                return -EINVAL;
        }
        if (val > 1) {
                val = 1;
        }

        tweaks_cpufreqctrlena_set_val(val);

        return count;
}

static DEVICE_ATTR(cpufqctlena, 0644, cpufreqctrl_show, cpufreqctrl_dump);

static int __init tweaks_init(void)
{
	int ret = 0;

	atomic_set(&ds_val, DEFAULT_DS_STATE);
	atomic_set(&zram_val, DEFAULT_ZRAM_OVERWRITE);
	atomic_set(&cpufreqctrl_val, DEFAULT_CPUFREQCTRL);

	if (tweaks_kfpobj == NULL) {
		tweaks_kfpobj = kobject_create_and_add("tweaks", NULL) ;
	}
	if (tweaks_kfpobj == NULL) {
		pr_warn("%s: tweaks_kobj create_and_add failed\n", __func__);
		ret = -ENODEV;
	}
	else {
		ret = sysfs_create_file(tweaks_kfpobj, &dev_attr_dynswappiness.attr);
		if (ret) {
                        pr_warn("%s: sysfs_create_file failed for dynswappiness\n", __func__);
                }
		ret = sysfs_create_file(tweaks_kfpobj, &dev_attr_zramoverwrite.attr);
		if (ret) {
			pr_warn("%s: sysfs_create_file failed for zramoverwrite\n", __func__);
		}
		ret = sysfs_create_file(tweaks_kfpobj, &dev_attr_cpufqctlena.attr);
		if (ret) {
                        pr_warn("%s: sysfs_create_file failed for cpufqctlena\n", __func__);
                }
	}

	return 0;
}

static void __exit tweaks_exit(void)
{
}

module_init(tweaks_init);
module_exit(tweaks_exit);

