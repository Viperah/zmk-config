#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(hall_debug, LOG_LEVEL_INF);

#define HALL_ADC_NODE DT_PATH(zephyr_user)

#if !DT_NODE_HAS_PROP(HALL_ADC_NODE, io_channels)
#error "Missing hall ADC io-channel in /zephyr,user"
#endif

static int16_t sample_buffer;
static const struct adc_dt_spec hall_adcs[] = {
    ADC_DT_SPEC_GET_BY_IDX(HALL_ADC_NODE, 0), /* Hall 1: D21 / P0.31 / AIN7 */
    ADC_DT_SPEC_GET_BY_IDX(HALL_ADC_NODE, 1), /* Hall 2: D20 / P0.29 / AIN5 */
};

static struct adc_sequence hall_sequence = {
    .buffer = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
};

static int setup_hall_adc(const struct adc_dt_spec *adc, const char *label)
{
    int err;

    if (!adc_is_ready_dt(adc)) {
        LOG_ERR("%s ADC device %s not ready", label, adc->dev->name);
        return -ENODEV;
    }

    err = adc_channel_setup_dt(adc);
    if (err) {
        LOG_ERR("%s adc_channel_setup_dt failed: %d", label, err);
        return err;
    }

    return 0;
}

static void log_hall_adc(const struct adc_dt_spec *adc, const char *label)
{
    int err;

    sample_buffer = 0;
    (void)adc_sequence_init_dt(adc, &hall_sequence);
    err = adc_read_dt(adc, &hall_sequence);

    if (err) {
        LOG_ERR("%s adc_read_dt failed: %d", label, err);
        return;
    }

    int32_t mv = sample_buffer;

    err = adc_raw_to_millivolts_dt(adc, &mv);

    if (err) {
        LOG_INF("%s raw=%d mv=ERR", label, sample_buffer);
    } else {
        LOG_INF("%s raw=%d mv=%d", label, sample_buffer, mv);
    }
}

static void hall_debug_thread(void)
{
    int err;

    k_sleep(K_SECONDS(2));

    err = setup_hall_adc(&hall_adcs[0], "hall1");
    if (err) {
        return;
    }

    err = setup_hall_adc(&hall_adcs[1], "hall2");
    if (err) {
        return;
    }

    LOG_INF("started hall1 on D21 / P0.31 / AIN7");
    LOG_INF("started hall2 on D20 / P0.29 / AIN5");

    while (1) {
        log_hall_adc(&hall_adcs[0], "hall1");
        log_hall_adc(&hall_adcs[1], "hall2");

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
