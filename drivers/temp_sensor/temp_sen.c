#define DT_DRV_COMPAT custom_temp_sensor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include "temp_sen.h"

/* Config: Read-only, stored in Flash (e.g., pin numbers, I2C addresses) */
struct mydevice_config {
    uint8_t i2c_address;
    uint32_t sampling_rate;
};

/* Data: Read/Write, stored in RAM (e.g., dynamic states, raw buffers, mutexes) */
struct mydevice_data {
    uint16_t latest_value;
    struct k_mutex lock;
};

//Initialization function called at the time of boot
static int temp_sensor_init(const struct device *dev)
{
    const struct mydevice_config *config = dev->config;
    struct mydevice_data *data = dev->data;

    // this is made simply to manage the telemetry data's flow during the normal functioning of the whatever device uses this driver
    k_mutex_init(&data->lock);
    
    /* Clear out our baseline data variable */
    data->latest_value = 0;

    /* In a real I2C driver, you would use the 'config->i2c_address' here to check if the chip is alive: */
    
    if (!device_is_ready(config->bus)) {
        LOG_ERR("Initialization failed for %s: Bus controller hardware is NOT ready!", dev->name);
        return -ENODEV;
    }
    
    LOG_INF("Temperature sensor %s successfully initialized!", dev->name);

    /* 4. Return 0 to tell Zephyr this device successfully initialized */
    return 0; 
}

// API functions to acctually fetch the temperature value from the sensor and return it to the application
static int gx600_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    const struct mydevice_config *config = dev->config;
    struct mydevice_data *data = dev->data;
    int ret;

    /* Ensure the user is requesting temperature data */
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /*Lock the memory to prevent race conditions */
    k_mutex_lock(&data->lock, K_FOREVER);

    /*Configure a standard Zephyr ADC read sequence */
    uint16_t adc_raw_buffer;
    struct adc_sequence sequence = {
        .buffer = &adc_raw_buffer,
        .buffer_size = sizeof(adc_raw_buffer),
    };
    
    /* Populate the sequence with Devicetree settings fetched during init */
    adc_sequence_init_dt(&config->adc_channel, &sequence);

    /*Fire the hardware conversion loop */
    ret = adc_read(config->adc_channel.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC read failed on device %s with error: %d", dev->name, ret);
        k_mutex_unlock(&data->lock);
        return ret;
    }

    data->latest_value = adc_raw_buffer;

    double v_out = (double)data->latest_value * (3.3 / 4095.0);
    if (v_out >= 3.3) v_out = 3.299;
    double r_ntc = (10000.0 * v_out) / (3.3 - v_out);
    double kelvin = r_ntc / 10000.0;             /* R / R25 */
    kelvin = log(kelvin);                         /* ln(R / R25) */
    kelvin /= 3950.0;                             /* 1/B * ln(R / R25) */
    kelvin += 1.0 / (25.0 + 273.15);              /* + 1/T25 (in Kelvin) */
    kelvin = 1.0 / kelvin;                        /* Invert to absolute Kelvin */

    data->calculated_temp = kelvin - 273.15;

    k_mutex_unlock(&data->lock);
    return 0;
}

static int gx600_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct mydevice_data *data = dev->data;

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /* Lock memory while parsing the float to prevent half-read race conditions */
    k_mutex_lock(&data->lock, K_FOREVER);

    /* Zephyr uses 'struct sensor_value' to avoid floating-point overhead in user apps.
       val1 = Integer Part, val2 = Fractional part in micro-degrees (1/1000000) */
    val->val1 = (int32_t)data->calculated_temp;
    val->val2 = (int32_t)((data->calculated_temp - val->val1) * 1000000);

    k_mutex_unlock(&data->lock);
    return 0;
}

static const struct sensor_driver_api gx600_api_funcs = {
    .sample_fetch = gx600_sample_fetch,
    .channel_get = gx600_channel_get,
};

#define MYDEVICE_INIT(inst)                                                             \
    static struct mydevice_data mydevice_data_##inst;                                   \
                                                                                        \
    static const struct mydevice_config mydevice_config_##inst = {                      \
        .i2c_address = DT_INST_REG_ADDR(inst),                                          \
        .sampling_rate = DT_INST_PROP(inst, sampling_rate),                             \
    };                                                                                  \
                                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                                         \
                          mydevice_init,             /* The init function */            \
                          NULL,                                                         \
                          &mydevice_data_##inst,                                        \
                          &mydevice_config_##inst,                                      \
                          POST_KERNEL,               /* <--- Boot Level */              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, /* <-- Boot Priority */   \
                          &gx600_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MYDEVICE_INIT)





