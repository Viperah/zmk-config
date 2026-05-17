#define DT_DRV_COMPAT diy_behavior_cpi_cycle

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <paw3395.h>
#include <zmk/behavior.h>

LOG_MODULE_REGISTER(diy_mouse_cpi_cycle, LOG_LEVEL_INF);

#define PAW3395_NODE DT_NODELABEL(paw3395)

static const struct device *const paw3395_dev = DEVICE_DT_GET(PAW3395_NODE);
static const uint16_t cpi_steps[] = {400, 600, 800, 1200, 1600};
static uint8_t cpi_index = 1;

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

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    cpi_index = (cpi_index + 1) % ARRAY_SIZE(cpi_steps);
    return set_cpi(cpi_steps[cpi_index]);
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    return 0;
}

static const struct behavior_driver_api behavior_cpi_cycle_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define CPI_CYCLE_INST(n)                                                                         \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_cpi_cycle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CPI_CYCLE_INST)
