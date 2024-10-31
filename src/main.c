#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include "LCD.h"

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define LCD_SCREEN_NODE DT_ALIAS(lcd_screen)


const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
static const struct i2c_dt_spec lcd_screen = I2C_DT_SPEC_GET(LCD_SCREEN_NODE);
static const struct device *dht11 = DEVICE_DT_GET(DT_NODELABEL(dht11));

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

int main(void) {
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    init_lcd(&lcd_screen);

    while (1) {
        int ret = sensor_sample_fetch(dht11);
        int err;
	    uint32_t count = 0;
	    uint16_t buf;
	    struct adc_sequence sequence = {
		    .buffer = &buf,
		    .buffer_size = sizeof(buf),
	    };

	    for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		    if (!adc_is_ready_dt(&adc_channels[i])) {
			    printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			    return 0;
		    }

		    err = adc_channel_setup_dt(&adc_channels[i]);
		    if (err < 0) {
			    printk("Could not setup channel #%d (%d)\n", i, err);
			    return 0;
		    }
	    }

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

        for (int k = 0; k < 2; k++) {
	        printk("ADC reading[%u]:\n", count++);
	        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		        int32_t val_mv;

		        printk("- %s, channel %d: ", adc_channels[i].dev->name, adc_channels[i].channel_id);

		        (void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		        err = adc_read_dt(&adc_channels[i], &sequence);
		        if (err < 0) {
			        printk("Could not read (%d)\n", err);
			        continue;
		        }

			
		        if (adc_channels[i].channel_cfg.differential) {
			        val_mv = (int32_t)((int16_t)buf);
		        } else {
			        val_mv = (int32_t)buf;
		        }
		        printk("%"PRId32, val_mv);
		        err = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
		
		        if (err < 0) {
			        printk(" (value in mV not available)\n");
		        } else {
			        printk(" = %"PRId32" mV\n", val_mv);
		        }
            }
        }

        k_sleep(K_SECONDS(10));
    }
    return 0;
}
