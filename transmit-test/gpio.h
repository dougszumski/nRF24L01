/*
 * GPIO user space helpers
 *
 * Copyright 2009 Analog Devices Inc.
 * Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Licensed under the GPL-2 or later
 */
 
#ifndef GPIO_H
#define GPIO_H
int gpio_export(unsigned gpio);
int gpio_unexport(unsigned gpio);
int gpio_dir_out(unsigned gpio);
int gpio_dir_in(unsigned gpio);
int gpio_value(unsigned gpio, unsigned value);
#endif
