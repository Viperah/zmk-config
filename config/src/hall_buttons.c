#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zmk/pointing/input_split.h>

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
    uint16_t input_code;
    int16_t baseline;
    int32_t filtered;
    uint8_t press_samples;
    uint8_t release_samples;
    bool pressed;
};

static struct hall_button hall_buttons[] = {
    {.label = "left", .input_code = INPUT_BTN_0},
    {.label = "right", .input_code = INPUT_BTN_1},
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
    hall_buttons[index].filtered = hall_buttons[index].baseline;

    LOG_INF("%s Hall baseline raw=%d", hall_buttons[index].label,
            hall_buttons[index].baseline);

    return 0;
}

static void send_hall_button(size_t index, bool pressed) {
    int err = zmk_input_split_report_peripheral_event(
        CONFIG_DIY_MOUSE_HALL_INPUT_SPLIT_REG, INPUT_EV_KEY, hall_buttons[index].input_code,
        pressed ? 1 : 0, true);
    if (err) {
        LOG_WRN("%s Hall split report failed: %d", hall_buttons[index].label, err);
    }
}

static void process_hall_button(size_t index) {
    struct hall_button *button = &hall_buttons[index];
    int16_t raw = 0;
    int err = read_hall_raw(&hall_adcs[index], &raw);
    if (err) {
        LOG_ERR("%s Hall read failed: %d", button->label, err);
        return;
    }

    if (CONFIG_DIY_MOUSE_HALL_FILTER_SHIFT > 0) {
        button->filtered +=
            (raw - button->filtered) >> CONFIG_DIY_MOUSE_HALL_FILTER_SHIFT;
    } else {
        button->filtered = raw;
    }

    int32_t delta = abs(button->filtered - button->baseline);

    if (!button->pressed && delta >= CONFIG_DIY_MOUSE_HALL_PRESS_DELTA) {
        button->press_samples++;
        button->release_samples = 0;
    } else if (button->pressed && delta <= CONFIG_DIY_MOUSE_HALL_RELEASE_DELTA) {
        button->release_samples++;
        button->press_samples = 0;
    } else {
        button->press_samples = 0;
        button->release_samples = 0;
    }

    if (!button->pressed && button->press_samples >= CONFIG_DIY_MOUSE_HALL_PRESS_SAMPLES) {
        button->pressed = true;
        button->press_samples = 0;
        LOG_INF("%s Hall pressed raw=%d filtered=%d delta=%d", button->label, raw,
                button->filtered, delta);
        send_hall_button(index, true);
    } else if (button->pressed &&
               button->release_samples >= CONFIG_DIY_MOUSE_HALL_RELEASE_SAMPLES) {
        button->pressed = false;
        button->release_samples = 0;
        LOG_INF("%s Hall released raw=%d filtered=%d delta=%d", button->label, raw,
                button->filtered, delta);
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

    LOG_INF("Hall buttons active: press_delta=%d release_delta=%d interval_us=%d filter_shift=%d",
            CONFIG_DIY_MOUSE_HALL_PRESS_DELTA, CONFIG_DIY_MOUSE_HALL_RELEASE_DELTA,
            CONFIG_DIY_MOUSE_HALL_SAMPLE_INTERVAL_US, CONFIG_DIY_MOUSE_HALL_FILTER_SHIFT);

    while (1) {
        for (size_t i = 0; i < ARRAY_SIZE(hall_buttons); i++) {
            process_hall_button(i);
        }

        k_sleep(K_USEC(CONFIG_DIY_MOUSE_HALL_SAMPLE_INTERVAL_US));
    }
}

K_THREAD_DEFINE(hall_buttons_tid, 1536, hall_buttons_thread, NULL, NULL, NULL, 7, 0, 0);
