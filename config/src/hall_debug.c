#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_saadc.h>

#define HALL_ADC_NODE DT_NODELABEL(adc)

/*
 * We use logical ADC channel 0,
 * but connect it to physical nRF SAADC input AIN7.
 *
 * nice!nano pin:
 * D21 / P0.31 / AIN7
 */
#define HALL_ADC_CHANNEL_ID 0
#define HALL_ADC_RESOLUTION 12

static const struct device *adc_dev = DEVICE_DT_GET(HALL_ADC_NODE);

static int16_t sample_buffer;

static struct adc_channel_cfg hall_channel_cfg = {
    .gain = ADC_GAIN_1_6,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
    .channel_id = HALL_ADC_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = NRF_SAADC_INPUT_AIN7,
#endif
};

static struct adc_sequence hall_sequence = {
    .channels = BIT(HALL_ADC_CHANNEL_ID),
    .buffer = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = HALL_ADC_RESOLUTION,
};

static void hall_debug_thread(void)
{
    int err;

    k_sleep(K_SECONDS(2));

    if (!device_is_ready(adc_dev)) {
        printk("hall_debug: ADC device not ready\n");
        return;
    }

    err = adc_channel_setup(adc_dev, &hall_channel_cfg);
    if (err) {
        printk("hall_debug: adc_channel_setup failed: %d\n", err);
        return;
    }

    printk("hall_debug: started on D21 / P0.31 / AIN7\n");

    while (1) {
        err = adc_read(adc_dev, &hall_sequence);

        if (err) {
            printk("hall0 adc_read failed: %d\n", err);
        } else {
            int32_t mv = sample_buffer;

            err = adc_raw_to_millivolts(
                adc_ref_internal(adc_dev),
                hall_channel_cfg.gain,
                HALL_ADC_RESOLUTION,
                &mv
            );

            if (err) {
                printk("hall0 raw=%d mv=ERR\n", sample_buffer);
            } else {
                printk("hall0 raw=%d mv=%d\n", sample_buffer, mv);
            }
        }

        /*
         * 50 ms = 20 logs per second.
         * Later we can sample at 1000 Hz internally,
         * but logging 1000 lines/s is not useful for debugging.
         */
        k_sleep(K_MSEC(50));
    }
}

K_THREAD_DEFINE(
    hall_debug_tid,
    1024,
    hall_debug_thread,
    NULL,
    NULL,
    NULL,
    7,
    0,
    0
);
