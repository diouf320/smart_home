/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include "lcd_screen_i2c.h"

#define LED_YELLOW_NODE DT_ALIAS(led_yellow)
#define LCD_SCREEN_NODE DT_ALIAS(lcd_screen)

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

const struct gpio_dt_spec led_yellow_gpio = GPIO_DT_SPEC_GET_OR(LED_YELLOW_NODE, gpios, {0});
static const struct i2c_dt_spec lcd_screen = I2C_DT_SPEC_GET(LCD_SCREEN_NODE);
const struct device *const dht11 = DEVICE_DT_GET_ONE(aosong_dht);

static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

int main(void)
{
    int ret;
    uint16_t adc_buf;
    uint32_t count = 0;
    struct sensor_value temp, humidity;
    struct adc_sequence sequence = {
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
    };

    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    gpio_pin_configure_dt(&led_yellow_gpio, GPIO_OUTPUT_HIGH);
    init_lcd(&lcd_screen);

    if (!device_is_ready(dht11)) {
        printk("Le capteur DHT11 n'est pas prêt\n");
        return;
    }

    for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
        if (!adc_is_ready_dt(&adc_channels[i])) {
            printk("Le périphérique ADC %s n'est pas prêt\n", adc_channels[i].dev->name);
            return;
        }

        ret = adc_channel_setup_dt(&adc_channels[i]);
        if (ret < 0) {
            printk("Échec de configuration du canal ADC #%d (%d)\n", i, ret);
            return;
        }
    }

    while (1) {
        
        ret = sensor_sample_fetch(dht11);
        if (ret == 0) {
            sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		    sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &humidity);

		    printf("Température: %.d.%06d °C, Humidité: %d.%06d %%\n", temp.val1, temp.val2, humidity.val1, humidity.val2);
        } else {
            printk("Erreur de lecture du capteur DHT11\n");
        }

        printk("Lecture ADC[%u]:\n", count++);
        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
            int32_t val_mv;
            printk("- %s, canal %d: ", adc_channels[i].dev->name, adc_channels[i].channel_id);

            (void)adc_sequence_init_dt(&adc_channels[i], &sequence);
            ret = adc_read_dt(&adc_channels[i], &sequence);
            if (ret < 0) {
                printk("Erreur de lecture ADC (%d)\n", ret);
                continue;
            }

            val_mv = adc_channels[i].channel_cfg.differential ? (int32_t)((int16_t)adc_buf) : (int32_t)adc_buf;
            printk("%" PRId32, val_mv);
            ret = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
            if (ret < 0) {
                printk(" (valeur en mV non disponible)\n");
            } else {
                printk(" = %" PRId32 " mV\n", val_mv);
            }
        }

        k_sleep(K_SECONDS(10));
    }
}

//	while(1){
//		r  = sensor_sample_fetch(dht11);
//		if (r < 0){
//			printf("Erreur");
//			continue;
//		}
//
//		sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp);
//		sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &humidity);
//
//		printf("Température: %.d.%06d °C, Humidité: %d.%06d %%\n", temp.val1, temp.val2, humidity.val1, humidity.val2);
//
//		k_sleep(K_SECONDS(10));
//	}