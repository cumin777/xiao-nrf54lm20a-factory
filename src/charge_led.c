#include "charge_led.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/mfd/npm13xx.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#ifndef CONFIG_FACTORY_CHARGE_LED_BAT_PRESENT_MIN_MV
#define CONFIG_FACTORY_CHARGE_LED_BAT_PRESENT_MIN_MV 2500
#endif

#define PMIC_NODE DT_NODELABEL(pmic)
#define PMIC_CHARGER_NODE DT_NODELABEL(pmic_charger)
#define PMIC_LEDS_NODE DT_NODELABEL(pmic_leds)

#define NPM1300_LED1 1U
#define NPM1300_CHGR_BASE 0x03U
#define NPM1300_BCHGCHARGESTATUS_OFFSET 0x34U

#define BCHG_STATUS_TRICKLECHARGE BIT(2)
#define BCHG_STATUS_CONSTANTCURRENT BIT(3)
#define BCHG_STATUS_CONSTANTVOLTAGE BIT(4)
#define BCHG_STATUS_ACTIVE_CHARGING_MASK \
	(BCHG_STATUS_TRICKLECHARGE | BCHG_STATUS_CONSTANTCURRENT | \
	 BCHG_STATUS_CONSTANTVOLTAGE)

#define CHARGE_LED_POLL_MS 500

#if IS_ENABLED(CONFIG_FACTORY_CHARGE_LED) && \
	DT_NODE_EXISTS(PMIC_NODE) && \
	DT_NODE_EXISTS(PMIC_CHARGER_NODE) && \
	DT_NODE_EXISTS(PMIC_LEDS_NODE)

static const struct device *const pmic_dev = DEVICE_DT_GET(PMIC_NODE);
static const struct device *const charger_dev = DEVICE_DT_GET(PMIC_CHARGER_NODE);
static const struct device *const pmic_led_dev = DEVICE_DT_GET(PMIC_LEDS_NODE);

static bool charge_led_ready;
static bool charge_led_on;
static int64_t last_charge_led_poll_ms;

static void charge_led_set(bool on)
{
	if (!charge_led_ready || charge_led_on == on) {
		return;
	}

	if (on) {
		(void)led_on(pmic_led_dev, NPM1300_LED1);
	} else {
		(void)led_off(pmic_led_dev, NPM1300_LED1);
	}

	charge_led_on = on;
}

static bool charge_led_battery_present(int32_t bat_mv)
{
	if (CONFIG_FACTORY_CHARGE_LED_BAT_PRESENT_MIN_MV <= 0) {
		return true;
	}

	return bat_mv >= CONFIG_FACTORY_CHARGE_LED_BAT_PRESENT_MIN_MV;
}

static int charge_led_read_state(bool *charging)
{
	struct sensor_value bat_voltage;
	uint8_t chg_status;
	int32_t bat_mv;
	int rc;

	if (charging == NULL) {
		return -EINVAL;
	}

	rc = sensor_sample_fetch(charger_dev);
	if (rc != 0) {
		return rc;
	}

	rc = sensor_channel_get(charger_dev, SENSOR_CHAN_GAUGE_VOLTAGE,
				&bat_voltage);
	if (rc != 0) {
		return rc;
	}

	rc = mfd_npm13xx_reg_read(pmic_dev, NPM1300_CHGR_BASE,
				  NPM1300_BCHGCHARGESTATUS_OFFSET,
				  &chg_status);
	if (rc != 0) {
		return rc;
	}

	bat_mv = (int32_t)(sensor_value_to_micro(&bat_voltage) / 1000);
	*charging = charge_led_battery_present(bat_mv) &&
		    ((chg_status & BCHG_STATUS_ACTIVE_CHARGING_MASK) != 0U);
	return 0;
}

void charge_led_init(void)
{
	if (!device_is_ready(pmic_dev) ||
	    !device_is_ready(charger_dev) ||
	    !device_is_ready(pmic_led_dev)) {
		charge_led_ready = false;
		return;
	}

	charge_led_ready = true;
	charge_led_on = true;
	charge_led_set(false);
}

void charge_led_poll(void)
{
	bool charging;
	int64_t now;

	if (!charge_led_ready) {
		return;
	}

	now = k_uptime_get();
	if (last_charge_led_poll_ms != 0 &&
	    (now - last_charge_led_poll_ms) < CHARGE_LED_POLL_MS) {
		return;
	}
	last_charge_led_poll_ms = now;

	if (charge_led_read_state(&charging) != 0) {
		charge_led_set(false);
		return;
	}

	charge_led_set(charging);
}

#else

void charge_led_init(void)
{
}

void charge_led_poll(void)
{
}

#endif
