/*
 *  include/linux/tweaks.h
 *
 *  Copyright (C) 2020 Cojocar Andrei
 *
 */
#ifndef __TWEAKS_H
#define __TWEAKS_H

#include <linux/notifier.h>

#define DEFAULT_DS_STATE 1
#define DEFAULT_VTG_LEVEL 5000

#define TWEAKS_VTG_EVENT       0x01
#define TWEAKS_ZRAM_EVENT      0x02
#define TWEAKS_CPUFQCTRL_EVENT 0x03
#define LCD_EVENT_ON           0x01
#define LCD_EVENT_OFF          0x02
#define LCD_EVENT_DOZE         0x03
#define LCD_EVENT_DOZE_SUSPEND 0x04

struct tcm_event {
       void *data;
};

extern unsigned int is_zramsize_overwritten(void);

extern int tweaks_register_client(struct notifier_block *nb);
extern int tweaks_unregister_client(struct notifier_block *nb);
extern int tweaks_notifier_call_chain(unsigned long val, void *data);

extern int lcd_register_client(struct notifier_block *nb);
extern int lcd_unregister_client(struct notifier_block *nb);
extern int lcd_notifier_call_chain(unsigned long val, void *data);

#endif

