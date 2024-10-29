/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "LCD.h"

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define LCD_SCREEN_NODE DT_ALIAS(lcd_screen)
#define I2C_DEVICE

const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
static const struct i2c_dt_spec lcd_screen = I2C_DT_SPEC_GET(LCD_SCREEN_NODE);

int main(void)
{
	gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	init_lcd(&lcd_screen);

	return 0;
}



