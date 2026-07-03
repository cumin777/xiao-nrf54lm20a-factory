#include "factory_status_led.h"

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>

#include "factory_storage.h"

#define PMIC_LEDS_NODE DT_NODELABEL(pmic_leds)
#define NPM1300_LED1 1U

#if IS_ENABLED(CONFIG_FACTORY_STATUS_LED) && DT_NODE_EXISTS(PMIC_LEDS_NODE)

static const struct device *const pmic_led_dev = DEVICE_DT_GET(PMIC_LEDS_NODE);
static bool factory_status_led_ready;
static bool factory_status_led_on;

static void factory_status_led_set(bool on)
{
	int rc;

	if (!factory_status_led_ready || factory_status_led_on == on) {
		return;
	}

	rc = on ? led_on(pmic_led_dev, NPM1300_LED1) :
		  led_off(pmic_led_dev, NPM1300_LED1);
	if (rc == 0) {
		factory_status_led_on = on;
	}
}

void factory_status_led_init(void)
{
	if (!device_is_ready(pmic_led_dev)) {
		factory_status_led_ready = false;
		return;
	}

	factory_status_led_ready = true;
	factory_status_led_on = true;
	factory_status_led_set(false);
}

void factory_status_led_update(uint32_t boot_flag)
{
	factory_status_led_set(boot_flag != FACTORY_BOOT_FLAG_ENTER_FACTORY);
}

#else

void factory_status_led_init(void)
{
}

void factory_status_led_update(uint32_t boot_flag)
{
	ARG_UNUSED(boot_flag);
}

#endif
