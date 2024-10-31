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
#include <zephyr/sys/time_units.h>  // Ajout pour mesurer le temps
#include "LCD.h"

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define LCD_SCREEN_NODE DT_ALIAS(lcd_screen)
#define BUTTON_1_NODE DT_ALIAS(button1)
#define BUTTON_2_NODE DT_ALIAS(button2)

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

const struct gpio_dt_spec button1_gpio = GPIO_DT_SPEC_GET_OR(BUTTON_1_NODE, gpios, {0});
const struct gpio_dt_spec button2_gpio = GPIO_DT_SPEC_GET_OR(BUTTON_2_NODE, gpios, {0});

volatile bool button1_pressed = false;
volatile bool button2_pressed = false;


#define DEBOUNCE_DELAY_MS 1000 
static int64_t last_press_time_button1 = 0;
static int64_t last_press_time_button2 = 0;

void button1_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int64_t now = k_uptime_get();  
    if ((now - last_press_time_button1) > DEBOUNCE_DELAY_MS) {
        button1_pressed = true;
        printk("Button 1 pressed\n");
        last_press_time_button1 = now;
    }
}

void button2_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int64_t now = k_uptime_get();  
    if ((now - last_press_time_button2) > DEBOUNCE_DELAY_MS) {
        button2_pressed = true;
        printk("Button 2 pressed\n");
        last_press_time_button2 = now;
    }
}

static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;

int main(void) {
    int ret;

    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);

    ret = gpio_pin_configure_dt(&button1_gpio, GPIO_INPUT);
    if (ret != 0) {
        printk("Erreur de configuration du bouton 1\n");
        return ret;
    }

    ret = gpio_pin_configure_dt(&button2_gpio, GPIO_INPUT);
    if (ret != 0) {
        printk("Erreur de configuration du bouton 2\n");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Erreur de configuration de l'interruption pour le bouton 1\n");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button2_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Erreur de configuration de l'interruption pour le bouton 2\n");
        return ret;
    }

    gpio_init_callback(&button1_cb_data, button1_handler, BIT(button1_gpio.pin));
    gpio_add_callback(button1_gpio.port, &button1_cb_data);

    gpio_init_callback(&button2_cb_data, button2_handler, BIT(button2_gpio.pin));
    gpio_add_callback(button2_gpio.port, &button2_cb_data);

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
		        err = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
		
		        if (err < 0) {
			        printk(" (value in mV not available)\n");
		        } else {
			        printk(" = %"PRId32" mV\n", val_mv);
		        }
            }
        }

        if (button1_pressed) {
            printk("Action pour bouton 1\n");
            write_lcd(&lcd_screen, "Bouton 1 pressed", LCD_LINE_1);
            k_sleep(K_SECONDS(2));
            write_lcd(&lcd_screen, "                ", LCD_LINE_1);
            button1_pressed = false;
        }

        if (button2_pressed) {
            printk("Action pour bouton 2\n");
            write_lcd(&lcd_screen, "Bouton 2 pressed", LCD_LINE_2);
            k_sleep(K_SECONDS(2));
            write_lcd(&lcd_screen, "                ", LCD_LINE_2);
            button2_pressed = false;
        }

        k_sleep(K_SECONDS(10));
    }

    return 0;
}
