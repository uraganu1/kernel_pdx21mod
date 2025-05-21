// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Andrei Cojocar
 */

#define pr_fmt(fmt) "cpu_freqctrl_simple: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/tweaks.h>

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "Andrei Cojocar <cojocar.andrei@gmail.com>"
#define DRIVER_DESCRIPTION "Cpu simple frequency controller"
#define CPUFREQCTRLDRV_NAME "cpufreqctrl"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[cpufreqctrl]: "

#define MAX_FREQ_TABLE 32
#define BATTERY_HIGH 90
#define BATTERY_LOW 25
#define BATTERY_STEP 15
#define TEMPERATURE_LOW 28000
#define TEMPERATURE_STEP 4000
#define SILVER_IDX_STEP 1
#define GOLD_IDX_STEP 2
#define PRIME_IDX_STEP 3
#define DEFAULT_POLL_MS 4096
#define START_DELAY_SEC 128

#define DEFAULT_STARTUP_STATE 0 //disable

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

struct freqctrl_zone {
	u8 silver_idx;
	u8 gold_idx;
	u8 prime_idx;
};

struct freqctrl_table {
	u32 gold[MAX_FREQ_TABLE];
	u32 prime[MAX_FREQ_TABLE];
	u32 silver[MAX_FREQ_TABLE];
	struct freqctrl_zone max_zone;
	u8 max_istep;
	u8 initfq;
};

struct freqctrl_drv {
	struct notifier_block tweaks_notif;
	struct notifier_block lcd_notif;
	struct notifier_block cpu_notif;
	struct delayed_work freqctrl_work;
	struct workqueue_struct *wq;
	struct freqctrl_zone curr_zone;
	struct freqctrl_table *fq_table;
	struct power_supply *batt_psy;
	atomic_t battery_step;
	atomic_t temp_step;
	atomic_t temp_low;
	u32 start_delay;
	atomic_t displaysupended;
	atomic_t enable;
	struct platform_device *pdev;
};

struct freqctrl_drv_platform_info {
	/* IRQ handler */
	//irqreturn_t (*irq_handler)(int irq, void *data);
	/* Special IRQ flags */
	unsigned int irq_flags;
	/* Bit location of switch */
	unsigned int bit;
	/* Symbolic name */
	const char *name;
};

extern int tweaks_register_client(struct notifier_block *nb);
extern int tweaks_unregister_client(struct notifier_block *nb);

extern int lcd_register_client(struct notifier_block *nb);
extern int lcd_unregister_client(struct notifier_block *nb);

static struct freqctrl_drv *g_fqdrv = NULL;

static int tweaks_notifier_cb(struct notifier_block *nb,
                               unsigned long event,
                               void *data)
{
	bool prev_enabled;
	struct freqctrl_drv *fdrv = container_of(nb, typeof(*fdrv), tweaks_notif);;
	prev_enabled = atomic_read(&fdrv->enable);

	if (event == TWEAKS_CPUFQCTRL_EVENT) {
		if ((fdrv != NULL) && (data != NULL)) {
			if (prev_enabled != *(bool *)data) {
				atomic_set(&fdrv->enable, *(bool *)data);
				if (prev_enabled) {
					// switch it off
					cancel_delayed_work_sync(&fdrv->freqctrl_work);
					// must go with highest frequency for all clusters
					pr_debug("dbg - freqctrl switch off driver\n");

				}
				else {
					// switch it on
					queue_delayed_work(fdrv->wq, &fdrv->freqctrl_work, fdrv->start_delay/16 * HZ);
					pr_debug("dbg - freqctrl switch on driver\n");
				}
			}
		}
	}
	return NOTIFY_OK;
}

static int lcd_notifier_cb(struct notifier_block *nb,
                           unsigned long event,
                           void *data)
{
	struct freqctrl_drv *fdrv = container_of(nb, typeof(*fdrv), lcd_notif);
	if (fdrv != NULL) {
		if ((event == LCD_EVENT_ON) || (event == LCD_EVENT_DOZE_SUSPEND)) {
			atomic_set(&fdrv->displaysupended, 0);
			pr_debug("dbg - freqctrl display on caught\n");
		}
		else {
			atomic_set(&fdrv->displaysupended, 1);
			pr_debug("dbg - freqctrl display off caught\n");
		}
	}
	return NOTIFY_OK;
}


static void update_online_cpu_policy(void)
{
	unsigned int cpu;
	u8 update_cpu = 0;

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		if (cpu_online(cpu)) {
			if (!(1 & update_cpu) && cpumask_intersects(cpumask_of(cpu), cpu_lp_mask)) {
				cpufreq_update_policy(cpu);
				update_cpu |= 1;
			}
			if (!(2 & update_cpu) && cpumask_intersects(cpumask_of(cpu), cpu_perf_mask)) {
				cpufreq_update_policy(cpu);
				update_cpu |= 2;
			}
			if (!(4 & update_cpu) && cpumask_intersects(cpumask_of(cpu), cpu_prime_mask)) {
				cpufreq_update_policy(cpu);
				update_cpu |= 4;
			}
		}
	}
	put_online_cpus();
}

static void freqctrl_throttle_worker(struct work_struct *work)
{
	struct freqctrl_drv *t = container_of(to_delayed_work(work), typeof(*t), freqctrl_work);
	union power_supply_propval psp = {0,};
	int battery_percentage = 100, temp = 0, temp_cpus_avg = 0, temp_batt = 0;
	s64 temp_total = 0, temp_avg = 0;
	short i = 0;
	int tmp_lvl = 0;
	u8 silver_idx = t->fq_table->max_zone.silver_idx;
	u8 gold_idx = t->fq_table->max_zone.gold_idx;
	u8 prime_idx = t->fq_table->max_zone.prime_idx;
	u8 temp_silver_idx = silver_idx;
	u8 temp_gold_idx = gold_idx;
	u8 temp_prime_idx = prime_idx;

	pr_debug("dbg - freqctrl, entering freqctrl_throttle_worker\n");

	/* Store average temperature of all CPU cores */
	for (i = 0; i < NR_CPUS; i++) {
		char zone_name[15];
		sprintf(zone_name, "cpu-1-%i-usr", i);
		thermal_zone_get_temp(thermal_zone_get_zone_by_name(zone_name), &temp);
		temp_total += temp;
	}

	temp_cpus_avg = temp_total / NR_CPUS;

	/* Now let's also get battery temperature */
	thermal_zone_get_temp(thermal_zone_get_zone_by_name("battery"), &temp_batt);

	if (!t->batt_psy) {
		t->batt_psy = power_supply_get_by_name("battery");
	}

	if (t->batt_psy && !power_supply_get_property(t->batt_psy,
						POWER_SUPPLY_PROP_CAPACITY,
						&psp)) {
		battery_percentage = psp.intval;
	}

	if (battery_percentage < BATTERY_LOW) {
		silver_idx = t->fq_table->max_zone.silver_idx/4;
		gold_idx = t->fq_table->max_zone.gold_idx/4;
		prime_idx = t->fq_table->max_zone.prime_idx/4;
	}
	else {
		i = 0;
		tmp_lvl = BATTERY_HIGH;
		while ((battery_percentage < tmp_lvl) && (i <= t->fq_table->max_istep)) {
			i++;
			tmp_lvl -= atomic_read(&t->battery_step);
		}
		if (i < t->fq_table->max_istep)
			i += atomic_read(&t->displaysupended);
		if (i > 0) {
			silver_idx -= i*SILVER_IDX_STEP ;
			gold_idx -= i*GOLD_IDX_STEP;
			prime_idx -= i*PRIME_IDX_STEP;
		}
	}

	/* HQ autism coming up */
	if (temp_batt <= 30000)
		temp_avg = (temp_cpus_avg * 2 + temp_batt * 3) / 5;
	else if (temp_batt > 30000 && temp_batt <= 38000)
		temp_avg = (temp_cpus_avg * 3 + temp_batt * 2) / 5;
	else if (temp_batt > 38000 && temp_batt <= 43000)
		temp_avg = (temp_cpus_avg * 4 + temp_batt) / 5;
	else if (temp_batt > 43000)
		temp_avg = (temp_cpus_avg * 5 + temp_batt) / 6;

	/* Emergency case */
	if (temp_cpus_avg > 90000)
		temp_avg = (temp_cpus_avg * 6 + temp_batt) / 7;

	tmp_lvl = atomic_read(&t->temp_low) - atomic_read(&t->displaysupended)*atomic_read(&t->temp_step);
	i = 0;

	while ((temp_avg > tmp_lvl) && (i <= t->fq_table->max_istep)) {
		i++;
		tmp_lvl += atomic_read(&t->temp_step);
	}
	if (i > 0) {
		temp_silver_idx -= i*(SILVER_IDX_STEP + 1);
		temp_gold_idx = (i > 1) ? temp_gold_idx - (i - 1)*(GOLD_IDX_STEP + 1) : temp_gold_idx - GOLD_IDX_STEP;
		temp_prime_idx -= i*PRIME_IDX_STEP;
	}

	silver_idx = temp_silver_idx < silver_idx ? temp_silver_idx : silver_idx;
	gold_idx = temp_gold_idx < gold_idx ? temp_gold_idx : gold_idx;
	prime_idx = temp_prime_idx < prime_idx ? temp_prime_idx : prime_idx;

	if ((silver_idx != t->curr_zone.silver_idx) ||
		(gold_idx != t->curr_zone.gold_idx) ||
		(prime_idx != t->curr_zone.prime_idx)) {
			t->curr_zone.silver_idx = silver_idx;
			t->curr_zone.gold_idx = gold_idx;
			t->curr_zone.prime_idx = prime_idx;
			pr_debug("dbg - freqctrl, updating cpu frequency\n");
			update_online_cpu_policy();
	}

	queue_delayed_work(t->wq, &t->freqctrl_work, msecs_to_jiffies((atomic_read(&t->displaysupended)+1)*DEFAULT_POLL_MS/(i+1)));
}

static u32 get_throttle_freq(struct freqctrl_drv *drv, u32 cpu)
{
	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		return drv->fq_table->silver[drv->curr_zone.silver_idx];
	else if (cpumask_test_cpu(cpu, cpu_perf_mask))
		return drv->fq_table->gold[drv->curr_zone.gold_idx];

	return drv->fq_table->prime[drv->curr_zone.prime_idx];
}

static int cpu_notifier_cb(struct notifier_block *nb,
                           unsigned long val,
                           void *data)
{
	struct freqctrl_drv *t = container_of(nb, typeof(*t), cpu_notif);
	struct cpufreq_policy *policy = data;
	u32 target_freq;

	if ((val != CPUFREQ_ADJUST) || (atomic_read(&t->enable) == 0)) {
		return NOTIFY_OK;
	}

	target_freq = get_throttle_freq(t, policy->cpu);

	if (target_freq < policy->max)
		policy->max = target_freq;

	if (policy->max < policy->min)
		policy->min = policy->max;

	return NOTIFY_OK;
}

static int cpu_freq_ctrl_parse_dt(struct freqctrl_drv *fd)
{
	int ret, i;
	unsigned int cpu;
	struct cpufreq_policy *policy;

	fd->fq_table = kzalloc(sizeof(struct freqctrl_table), GFP_KERNEL);

	memset(fd->fq_table, 0, sizeof(struct freqctrl_table));

	pr_debug("dbg - freqctrl, entering cpu_freq_ctrl_parse_dt\n");

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		if (cpu_online(cpu)) {
			if (!(1 & fd->fq_table->initfq) && cpumask_intersects(cpumask_of(cpu), cpu_lp_mask)) {
				policy = cpufreq_cpu_get(cpu);
				ret = cpufreq_frequency_table_get_table(policy, fd->fq_table->silver);
				if ((ret > 0) && (ret  < MAX_FREQ_TABLE)) {
					fd->fq_table->initfq |= 1;
					fd->fq_table->max_zone.silver_idx = ret - 1;
				}
				else
					goto free_fqtable;
			}
			if (!(2 & fd->fq_table->initfq) && cpumask_intersects(cpumask_of(cpu), cpu_perf_mask)) {
				policy = cpufreq_cpu_get(cpu);
				ret = cpufreq_frequency_table_get_table(policy, fd->fq_table->gold);
				if ((ret > 0) && (ret  < MAX_FREQ_TABLE)) {
					fd->fq_table->initfq |= 2;
					fd->fq_table->max_zone.gold_idx = ret - 1;
				}
				else
					goto free_fqtable;
			}
			if (!(4 & fd->fq_table->initfq) && cpumask_intersects(cpumask_of(cpu), cpu_prime_mask)) {
				policy = cpufreq_cpu_get(cpu);
				ret = cpufreq_frequency_table_get_table(policy, fd->fq_table->prime);
				if ((ret > 0) && (ret  < MAX_FREQ_TABLE)) {
					fd->fq_table->initfq |= 4;
					fd->fq_table->max_zone.prime_idx = ret - 1;
				}
				else
					goto free_fqtable;
			}
		}
	}
	put_online_cpus();

	//dbg freq table
	for (i = 0; i < fd->fq_table->max_zone.silver_idx; i++) {
		pr_debug("dbg - freqctrl, cpu_silver.freq[%d] = %d\n", i, fd->fq_table->silver[i]);
	}
	for (i = 0; i < fd->fq_table->max_zone.gold_idx; i++) {
                pr_debug("dbg - freqctrl, cpu_gold.freq[%d] = %d\n", i, fd->fq_table->gold[i]);
        }
	for (i = 0; i < fd->fq_table->max_zone.prime_idx; i++) {
                pr_debug("dbg - freqctrl, cpu_prime.freq[%d] = %d\n", i, fd->fq_table->prime[i]);
        }
	//dbg ends

	fd->curr_zone.silver_idx = fd->fq_table->max_zone.silver_idx;
	fd->curr_zone.gold_idx = fd->fq_table->max_zone.gold_idx;
	fd->curr_zone.prime_idx = fd->fq_table->max_zone.prime_idx;
	atomic_set(&fd->enable, DEFAULT_STARTUP_STATE);
	atomic_set(&fd->battery_step, BATTERY_STEP);
	atomic_set(&fd->temp_step, TEMPERATURE_STEP);
	atomic_set(&fd->temp_low, TEMPERATURE_LOW);
	atomic_set(&fd->displaysupended, 0);

	fd->fq_table->max_istep = fd->fq_table->max_zone.silver_idx / SILVER_IDX_STEP;
	fd->fq_table->max_istep = fd->fq_table->max_zone.gold_idx / GOLD_IDX_STEP < fd->fq_table->max_istep ? fd->fq_table->max_zone.gold_idx / GOLD_IDX_STEP : fd->fq_table->max_istep;
	fd->fq_table->max_istep = fd->fq_table->max_zone.prime_idx / PRIME_IDX_STEP < fd->fq_table->max_istep ? fd->fq_table->max_zone.prime_idx / PRIME_IDX_STEP : fd->fq_table->max_istep;

	fd->batt_psy = power_supply_get_by_name("battery");

	fd->start_delay = START_DELAY_SEC;

	return 0;

free_fqtable:
	kfree(fd->fq_table);
	fd->fq_table = NULL;
	return ret;
}

static ssize_t batt_step_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	size_t count = 0;
	int val = 0;
	struct freqctrl_drv *fcd = g_fqdrv;

	if (fcd != NULL) {
		val = atomic_read(&fcd->battery_step);
	}

	count += sprintf(buf, "%d\n", val);
	return count;
}

static ssize_t batt_step_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int r, val;
	struct freqctrl_drv *fcd = g_fqdrv;

	r = kstrtoint(buf, 10, &val);
	if ((r) || (val < 0)) {
		return -EINVAL;
	}
	if (val < 5) {
		val = 5;
	}
	if (val > 50) {
		val = 50;
	}

	if (fcd != NULL) {
		atomic_set(&fcd->battery_step, val);
	}

	return count;
}

static DEVICE_ATTR_RW(batt_step);

static ssize_t temp_step_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
        size_t count = 0;
        int val = 0;
	struct freqctrl_drv *fcd = g_fqdrv;

        if (fcd != NULL) {
            val = atomic_read(&fcd->temp_step);
        }

        count += sprintf(buf, "%d\n", val);
        return count;
}

static ssize_t temp_step_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int r, val;
	struct freqctrl_drv *fcd = g_fqdrv;

	r = kstrtoint(buf, 10, &val);
	if ((r) || (val < 0)) {
		return -EINVAL;
	}
	if (val < 1000) {
		val = 1000;
	}
	if (val > 50000) {
		val = 50000;
	}

	if (fcd != NULL) {
		atomic_set(&fcd->temp_step, val);
	}

	return count;
}

static DEVICE_ATTR_RW(temp_step);

static ssize_t temp_low_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	size_t count = 0;
	int val = 0;
	struct freqctrl_drv *fcd = g_fqdrv;

	if (fcd != NULL) {
		val = atomic_read(&fcd->temp_low);
	}

	count += sprintf(buf, "%d\n", val);
	return count;
}

static ssize_t temp_low_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int r, val;
	struct freqctrl_drv *fcd = g_fqdrv;

	r = kstrtoint(buf, 10, &val);
	if ((r) || (val < 0)) {
		return -EINVAL;
	}
	if (val < 20000) {
		val = 20000;
	}
	if (val > 50000) {
		val = 50000;
	}

	if (fcd != NULL) {
		atomic_set(&fcd->temp_low, val);
	}

	return count;
}

static DEVICE_ATTR_RW(temp_low);

static const struct device_attribute *cpufreqctrl_attr_list[] = {
                &dev_attr_batt_step,
                &dev_attr_temp_step,
                &dev_attr_temp_low,
                NULL
};

static int cpufreqctrl_drv_probe(struct platform_device *pdev)
{
	struct freqctrl_drv *fccdrv;
	struct freqctrl_drv_platform_info *fccdrv_info;
	int ret, i;

	fccdrv = kzalloc(sizeof(struct freqctrl_drv), GFP_KERNEL);
	if (unlikely(!fccdrv))
		return -ENOMEM;
	
	fccdrv_info = pdev->dev.platform_data;
	BUG_ON(!fccdrv_info);

	if (fccdrv_info->name) {
		for (i = 0; cpufreqctrl_attr_list[i] != NULL; i++) {
			ret = device_create_file(&pdev->dev, cpufreqctrl_attr_list[i]);
			if (unlikely(ret)) {
                        	dev_err(&pdev->dev, "Failed creating device attrs\n");
                        	ret = -EINVAL;
                        	goto err_probe;
			}
		}
	}

	fccdrv->wq = alloc_workqueue("cpufreqctrl", WQ_POWER_EFFICIENT, 0);

	if (!fccdrv->wq) {
                ret = -ENOMEM;
                goto err_probe;
        }

	ret = cpu_freq_ctrl_parse_dt(fccdrv);
        if (ret)
                goto err_parsedt;

	/* Set the priority to INT_MIN so throttling can't be tampered with */
        fccdrv->cpu_notif.notifier_call = cpu_notifier_cb;
        fccdrv->cpu_notif.priority = INT_MIN;
        ret = cpufreq_register_notifier(&fccdrv->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
        if (ret) {
                pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
                goto err_zones;
        }

        fccdrv->tweaks_notif.notifier_call = tweaks_notifier_cb;
        ret = tweaks_register_client(&fccdrv->tweaks_notif);
        if (ret) {
                pr_err("Failed to register tweaks notifier, err: %d\n", ret);
        }

        fccdrv->lcd_notif.notifier_call = lcd_notifier_cb;
        ret = lcd_register_client(&fccdrv->lcd_notif);
        if (ret) {
                pr_err("Failed to register lcd notifier, err: %d\n", ret);
        }

        INIT_DELAYED_WORK(&fccdrv->freqctrl_work, freqctrl_throttle_worker);
	/* Fire up the persistent worker only through tweaks sysfs */
	if (DEFAULT_STARTUP_STATE) // disabled by default
        	queue_delayed_work(fccdrv->wq, &fccdrv->freqctrl_work, fccdrv->start_delay * HZ);

	fccdrv->pdev = pdev;
	g_fqdrv = fccdrv;
	platform_set_drvdata(pdev, fccdrv);

	return 0;

err_zones:
        kfree(fccdrv->fq_table);

err_parsedt:
	destroy_workqueue(fccdrv->wq);

err_probe:
        kfree(fccdrv);
        return ret;
}

static int cpufreqctrl_drv_remove(struct platform_device *pdev)
{
	int i;
	struct freqctrl_drv *fcd = platform_get_drvdata(pdev);
	struct freqctrl_drv_platform_info *fcd_info = pdev->dev.platform_data;

	if (fcd_info->name) {
		for (i = 0; cpufreqctrl_attr_list[i] != NULL; i++) {
                        device_remove_file(&pdev->dev, cpufreqctrl_attr_list[i]);
                }
	}

	cancel_delayed_work_sync(&fcd->freqctrl_work);
	cpufreq_unregister_notifier(&fcd->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	tweaks_unregister_client(&fcd->tweaks_notif);
	lcd_unregister_client(&fcd->lcd_notif);
	if(fcd->fq_table != NULL)
		kfree(fcd->fq_table);
	destroy_workqueue(fcd->wq);
	kfree(fcd);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver cpufreqctrl_driver = {
	.probe          = cpufreqctrl_drv_probe,
	.remove         = cpufreqctrl_drv_remove,
	.driver         = {
		.name   = CPUFREQCTRLDRV_NAME,
        },
};

static int __init cpufreqctrl_init(void)
{
        printk(KERN_NOTICE CPUFREQCTRLDRV_NAME ": version %s loaded\n", DRIVER_VERSION);
        return platform_driver_register(&cpufreqctrl_driver);
}

static void __exit cpufreqctrl_exit(void)
{
        platform_driver_unregister(&cpufreqctrl_driver);
}

module_init(cpufreqctrl_init);
module_exit(cpufreqctrl_exit);
