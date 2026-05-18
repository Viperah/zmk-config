#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <errno.h>
#include <paw3395.h>

LOG_MODULE_REGISTER(diy_mouse_digital_inputs, LOG_LEVEL_INF);

#define DIGITAL_BUTTONS_NODE DT_NODELABEL(digital_buttons_input)
#define SCROLL_WHEEL_NODE DT_NODELABEL(scroll_wheel_input)
#define PAW3395_NODE DT_NODELABEL(paw3395)

#define DIGITAL_BUTTON_COUNT DT_PROP_LEN(DIGITAL_BUTTONS_NODE, input_gpios)

BUILD_ASSERT(DIGITAL_BUTTON_COUNT == DT_PROP_LEN(DIGITAL_BUTTONS_NODE, button_codes),
             "digital button GPIO count must match button code count");

static int digital_buttons_input_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static int scroll_wheel_input_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

DEVICE_DT_DEFINE(DIGITAL_BUTTONS_NODE, digital_buttons_input_init, NULL, NULL, NULL, POST_KERNEL,
                 CONFIG_INPUT_INIT_PRIORITY, NULL);
DEVICE_DT_DEFINE(SCROLL_WHEEL_NODE, scroll_wheel_input_init, NULL, NULL, NULL, POST_KERNEL,
                 CONFIG_INPUT_INIT_PRIORITY, NULL);

static const struct device *const digital_buttons_dev = DEVICE_DT_GET(DIGITAL_BUTTONS_NODE);
static const struct device *const scroll_wheel_dev = DEVICE_DT_GET(SCROLL_WHEEL_NODE);
static const struct device *const paw3395_dev = DEVICE_DT_GET(PAW3395_NODE);

#define BUTTON_GPIO(idx, _) GPIO_DT_SPEC_GET_BY_IDX(DIGITAL_BUTTONS_NODE, input_gpios, idx)
#define BUTTON_CODE(idx, _) DT_PROP_BY_IDX(DIGITAL_BUTTONS_NODE, button_codes, idx)

static const struct gpio_dt_spec button_gpios[] = {
    LISTIFY(DIGITAL_BUTTON_COUNT, BUTTON_GPIO, (, )),
};

static const uint16_t button_codes[] = {
    LISTIFY(DIGITAL_BUTTON_COUNT, BUTTON_CODE, (, )),
};

static const struct gpio_dt_spec dpi_gpio = GPIO_DT_SPEC_GET(DIGITAL_BUTTONS_NODE, dpi_gpios);
static const struct gpio_dt_spec scroll_a_gpio = GPIO_DT_SPEC_GET(SCROLL_WHEEL_NODE, a_gpios);
static const struct gpio_dt_spec scroll_b_gpio = GPIO_DT_SPEC_GET(SCROLL_WHEEL_NODE, b_gpios);

struct debounced_button {
    bool stable_pressed;
    bool candidate_pressed;
    uint8_t samples;
};

static struct debounced_button buttons[DIGITAL_BUTTON_COUNT];
static struct debounced_button dpi_button;
static const uint16_t cpi_steps[] = {400, 600, 800, 1200, 1600};
static uint8_t cpi_index = 1;
static uint8_t scroll_last_state;
static int8_t scroll_accum;

static int configure_input_gpio(const struct gpio_dt_spec *gpio, const char *label) {
    if (!gpio_is_ready_dt(gpio)) {
        LOG_ERR("%s GPIO port is not ready", label);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("%s gpio_pin_configure_dt failed: %d", label, err);
    }

    return err;
}

static bool read_pressed(const struct gpio_dt_spec *gpio) {
    int value = gpio_pin_get_dt(gpio);
    return value > 0;
}

static int set_cpi(uint16_t cpi) {
    if (!device_is_ready(paw3395_dev)) {
        LOG_WRN("PAW3395 is not ready for CPI change");
        return -ENODEV;
    }

    struct sensor_value value = {
        .val1 = cpi,
        .val2 = 0,
    };

    int err = sensor_attr_set(paw3395_dev, SENSOR_CHAN_ALL,
                              (enum sensor_attribute)PAW3395_ATTR_CPI, &value);
    if (err) {
        LOG_WRN("PAW3395 CPI change to %u failed: %d", cpi, err);
        return err;
    }

    LOG_INF("PAW3395 CPI set to %u", cpi);
    return 0;
}

static bool debounce_update(struct debounced_button *button, bool pressed) {
    if (pressed != button->candidate_pressed) {
        button->candidate_pressed = pressed;
        button->samples = 1;
        return false;
    }

    if (button->samples < CONFIG_DIY_MOUSE_DIGITAL_DEBOUNCE_SAMPLES) {
        button->samples++;
    }

    if (button->samples >= CONFIG_DIY_MOUSE_DIGITAL_DEBOUNCE_SAMPLES &&
        button->stable_pressed != pressed) {
        button->stable_pressed = pressed;
        return true;
    }

    return false;
}

static void process_buttons(void) {
    for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
        bool pressed = read_pressed(&button_gpios[i]);
        if (debounce_update(&buttons[i], pressed)) {
            int err = input_report(digital_buttons_dev, INPUT_EV_KEY, button_codes[i],
                                   pressed ? 1 : 0, true, K_NO_WAIT);
            if (err) {
                LOG_WRN("Button %u input report failed: %d", button_codes[i], err);
            }
        }
    }

    bool dpi_pressed = read_pressed(&dpi_gpio);
    if (debounce_update(&dpi_button, dpi_pressed) && dpi_button.stable_pressed) {
        cpi_index = (cpi_index + 1) % ARRAY_SIZE(cpi_steps);
        (void)set_cpi(cpi_steps[cpi_index]);
    }
}

static uint8_t read_scroll_state(void) {
    uint8_t a = gpio_pin_get_dt(&scroll_a_gpio) > 0 ? 1 : 0;
    uint8_t b = gpio_pin_get_dt(&scroll_b_gpio) > 0 ? 1 : 0;

    return (a << 1) | b;
}

static void process_scroll(void) {
    static const int8_t transition_table[16] = {
        0,  -1, 1,  0,
        1,  0,  0,  -1,
        -1, 0,  0,  1,
        0,  1,  -1, 0,
    };

    uint8_t state = read_scroll_state();
    int8_t movement = transition_table[(scroll_last_state << 2) | state];
    scroll_last_state = state;

    if (movement == 0) {
        return;
    }

    scroll_accum += movement;
    if (scroll_accum >= CONFIG_DIY_MOUSE_SCROLL_EDGES_PER_REPORT) {
        scroll_accum = 0;
        (void)input_report(scroll_wheel_dev, INPUT_EV_REL, INPUT_REL_WHEEL,
                           CONFIG_DIY_MOUSE_SCROLL_STEP, true, K_NO_WAIT);
    } else if (scroll_accum <= -CONFIG_DIY_MOUSE_SCROLL_EDGES_PER_REPORT) {
        scroll_accum = 0;
        (void)input_report(scroll_wheel_dev, INPUT_EV_REL, INPUT_REL_WHEEL,
                           -CONFIG_DIY_MOUSE_SCROLL_STEP, true, K_NO_WAIT);
    }
}

static void digital_inputs_thread(void) {
    for (size_t i = 0; i < ARRAY_SIZE(button_gpios); i++) {
        int err = configure_input_gpio(&button_gpios[i], "digital button");
        if (err) {
            return;
        }

        buttons[i].stable_pressed = read_pressed(&button_gpios[i]);
        buttons[i].candidate_pressed = buttons[i].stable_pressed;
    }

    if (configure_input_gpio(&dpi_gpio, "DPI button") ||
        configure_input_gpio(&scroll_a_gpio, "scroll A") ||
        configure_input_gpio(&scroll_b_gpio, "scroll B")) {
        return;
    }

    dpi_button.stable_pressed = read_pressed(&dpi_gpio);
    dpi_button.candidate_pressed = dpi_button.stable_pressed;
    scroll_last_state = read_scroll_state();

    LOG_INF("Digital inputs active: buttons=%d debounce_samples=%d scroll_edges=%d scroll_step=%d",
            DIGITAL_BUTTON_COUNT, CONFIG_DIY_MOUSE_DIGITAL_DEBOUNCE_SAMPLES,
            CONFIG_DIY_MOUSE_SCROLL_EDGES_PER_REPORT, CONFIG_DIY_MOUSE_SCROLL_STEP);

    while (1) {
        process_buttons();
        process_scroll();
        k_sleep(K_USEC(CONFIG_DIY_MOUSE_DIGITAL_SAMPLE_INTERVAL_US));
    }
}

K_THREAD_DEFINE(digital_inputs_tid, 1536, digital_inputs_thread, NULL, NULL, NULL, 7, 0, 0);
