#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include "LCD.h"

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define LCD_SCREEN_NODE DT_ALIAS(lcd_screen)

const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
static const struct i2c_dt_spec lcd_screen = I2C_DT_SPEC_GET(LCD_SCREEN_NODE);
static const struct device *dht11 = DEVICE_DT_GET(DT_NODELABEL(dht11));

int main(void) {
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    init_lcd(&lcd_screen);

    while (1) {
        int ret = sensor_sample_fetch(dht11);
        if (ret == 0) {
            printk("Tout va bien\n");
        } 
        else {
            printk("Erreur\n");
            return -1;
        }

        struct sensor_value temp, hum;

        sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &hum);
    
        printf("Température: %d.%06d °C, Humidité: %d.%06d %%\n", temp.val1, temp.val2, hum.val1, hum.val2);
        k_sleep(K_SECONDS(10));
    }

    return 0;
}



