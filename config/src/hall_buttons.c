#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/endpoints.h>
#include <zmk/hid.h>

#include <errno.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(diy_mouse_hall_buttons, LOG_LEVEL_INF);

#define HALL_ADC_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_NODE_HAS_PROP(HALL_ADC_NODE, io_channels),
             "Missing Hall ADC io-channels in /zephyr,user");
BUILD_ASSERT(DT_PROP_LEN(HALL_ADC_NODE, io_channels) >= 2,
             "Two Hall ADC io-channels are required");

static const struct adc_dt_spec hall_adcs[] = {
    ADC_DT_SPEC_GET_BY_IDX(HALL_ADC_NODE, 0), /* Hall 1: D21 / P0.31 / AIN7 */
    ADC_DT_SPEC_GET_BY_IDX(HALL_ADC_NODE, 1), /* Hall 2: D20 / P0.29 / AIN5 */
};

struct hall_button {
    const char *label;
    zmk_mouse_button_t mouse_button;
    int16_t baseline;
    bool pressed;
};

static struct hall_button hall_buttons[] = {
    {.label = "left", .mouse_button = 0},
    {.label = "right", .mouse_button = 1},
};

static int read_hall_raw(const struct adc_dt_spec *adc, int16_t *raw) {
    int16_t sample_buffer = 0;
    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
    };

    (void)adc_sequence_init_dt(adc, &sequence);

    int err = adc_read_dt(adc, &sequence);
    if (err) {
        return err;
    }

    *raw = sample_buffer;
    return 0;
}

static int setup_hall_adc(const struct adc_dt_spec *adc, const char *label) {
    if (!adc_is_ready_dt(adc)) {
        LOG_ERR("%s Hall ADC device %s not ready", label, adc->dev->name);
        return -ENODEV;
    }

    int err = adc_channel_setup_dt(adc);
    if (err) {
        LOG_ERR("%s Hall adc_channel_setup_dt failed: %d", label, err);
    }

    return err;
}

static int calibrate_hall_button(size_t index) {
    int32_t sum = 0;

    for (int i = 0; i < CONFIG_DIY_MOUSE_HALL_BOOT_CALIBRATION_SAMPLES; i++) {
        int16_t raw = 0;
        int err = read_hall_raw(&hall_adcs[index], &raw);
        if (err) {
            LOG_ERR("%s Hall calibration read failed: %d", hall_buttons[index].label, err);
            return err;
        }

        sum += raw;
        k_sleep(K_USEC(CONFIG_DIY_MOUSE_HALL_SAMPLE_INTERVAL_US));
    }

    hall_buttons[index].baseline =
        (int16_t)(sum / CONFIG_DIY_MOUSE_HALL_BOOT_CALIBRATION_SAMPLES);

    LOG_INF("%s Hall baseline raw=%d", hall_buttons[index].label,
            hall_buttons[index].baseline);

    return 0;
}

static void send_hall_button(size_t index, bool pressed) {
    int err;

    if (pressed) {
        err = zmk_hid_mouse_button_press(hall_buttons[index].mouse_button);
    } else {
        err = zmk_hid_mouse_button_release(hall_buttons[index].mouse_button);
    }

    if (err) {
        LOG_ERR("%s Hall mouse button %s failed: %d", hall_buttons[index].label,
                pressed ? "press" : "release", err);
        return;
    }

    err = zmk_endpoint_send_mouse_report();
    if (err) {
        LOG_WRN("%s Hall mouse report send failed: %d", hall_buttons[index].label, err);
    }
}

static void process_hall_button(size_t index) {
    int16_t raw = 0;
    int err = read_hall_raw(&hall_adcs[index], &raw);
    if (err) {
        LOG_ERR("%s Hall read failed: %d", hall_buttons[index].label, err);
        return;
    }

    int32_t delta = abs(raw - hall_buttons[index].baseline);

    if (!hall_buttons[index].pressed && delta >= CONFIG_DIY_MOUSE_HALL_PRESS_DELTA) {
        hall_buttons[index].pressed = true;
        LOG_INF("%s Hall pressed raw=%d delta=%d", hall_buttons[index].label, raw, delta);
        send_hall_button(index, true);
    } else if (hall_buttons[index].pressed && delta <= CONFIG_DIY_MOUSE_HALL_RELEASE_DELTA) {
        hall_buttons[index].pressed = false;
        LOG_INF("%s Hall released raw=%d delta=%d", hall_buttons[index].label, raw, delta);
        send_hall_button(index, false);
    }
}

static void hall_buttons_thread(void) {
    LOG_INF("Hall button calibration starts in %d ms; leave main buttons released",
            CONFIG_DIY_MOUSE_HALL_BOOT_CALIBRATION_DELAY_MS);

    k_sleep(K_MSEC(CONFIG_DIY_MOUSE_HALL_BOOT_CALIBRATION_DELAY_MS));

    for (size_t i = 0; i < ARRAY_SIZE(hall_adcs); i++) {
        int err = setup_hall_adc(&hall_adcs[i], hall_buttons[i].label);
        if (err) {
            return;
        }
    }

    for (size_t i = 0; i < ARRAY_SIZE(hall_buttons); i++) {
        int err = calibrate_hall_button(i);
        if (err) {
            return;
        }
    }

    LOG_INF("Hall buttons active: press_delta=%d release_delta=%d interval_us=%d",
            CONFIG_DIY_MOUSE_HALL_PRESS_DELTA, CONFIG_DIY_MOUSE_HALL_RELEASE_DELTA,
            CONFIG_DIY_MOUSE_HALL_SAMPLE_INTERVAL_US);

    while (1) {
        for (size_t i = 0; i < ARRAY_SIZE(hall_buttons); i++) {
            process_hall_button(i);
        }

        k_sleep(K_USEC(CONFIG_DIY_MOUSE_HALL_SAMPLE_INTERVAL_US));
    }
}

K_THREAD_DEFINE(hall_buttons_tid, 1536, hall_buttons_thread, NULL, NULL, NULL, 7, 0, 0);
