#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hall_debug, LOG_LEVEL_INF);

#define HALL_ADC_NODE DT_PATH(zephyr_user)

#if !DT_NODE_HAS_PROP(HALL_ADC_NODE, io_channels)
#error "Missing hall ADC io-channel in /zephyr,user"
#endif

static int16_t sample_buffer;
static const struct adc_dt_spec hall_adc = ADC_DT_SPEC_GET(HALL_ADC_NODE);

static struct adc_sequence hall_sequence = {
    .buffer = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
};

static void hall_debug_thread(void)
{
    int err;

    k_sleep(K_SECONDS(2));

    if (!adc_is_ready_dt(&hall_adc)) {
        LOG_ERR("ADC device %s not ready", hall_adc.dev->name);
        return;
    }

    err = adc_channel_setup_dt(&hall_adc);
    if (err) {
        LOG_ERR("adc_channel_setup_dt failed: %d", err);
        return;
    }

    LOG_INF("started on D21 / P0.31 / AIN7");

    while (1) {
        sample_buffer = 0;
        (void)adc_sequence_init_dt(&hall_adc, &hall_sequence);
        err = adc_read_dt(&hall_adc, &hall_sequence);

        if (err) {
            LOG_ERR("hall0 adc_read_dt failed: %d", err);
        } else {
            int32_t mv = sample_buffer;

            err = adc_raw_to_millivolts_dt(&hall_adc, &mv);

            if (err) {
                LOG_INF("hall0 raw=%d mv=ERR", sample_buffer);
            } else {
                LOG_INF("hall0 raw=%d mv=%d", sample_buffer, mv);
            }
        }

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
