#include "at_handler.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>

#include "factory_storage.h"

#ifndef CONFIG_FACTORY_VBUS_USB_MIN_MV
#define CONFIG_FACTORY_VBUS_USB_MIN_MV 4900
#endif

#ifndef CONFIG_FACTORY_3V3_MIN_MV
#define CONFIG_FACTORY_3V3_MIN_MV 3200
#endif

#ifndef CONFIG_FACTORY_VBUS_BAT_MAX_MV
#define CONFIG_FACTORY_VBUS_BAT_MAX_MV 500
#endif

#ifndef CONFIG_FACTORY_ADC_VBUS_CHANNEL
#define CONFIG_FACTORY_ADC_VBUS_CHANNEL 0
#endif

#ifndef CONFIG_FACTORY_ADC_3V3_CHANNEL
#define CONFIG_FACTORY_ADC_3V3_CHANNEL 1
#endif

#ifndef CONFIG_FACTORY_ADC_BATV_CHANNEL
#define CONFIG_FACTORY_ADC_BATV_CHANNEL 2
#endif

#ifndef CONFIG_FACTORY_CHGCUR_REF_MA
#define CONFIG_FACTORY_CHGCUR_REF_MA 0
#endif

#ifndef CONFIG_FACTORY_BATV_DELTA_MAX_MV
#define CONFIG_FACTORY_BATV_DELTA_MAX_MV 200
#endif

#ifndef CONFIG_FACTORY_SLEEPI_WINDOW_MS
#define CONFIG_FACTORY_SLEEPI_WINDOW_MS 1000
#endif

#ifndef CONFIG_FACTORY_SLEEPI_REF_UA
#define CONFIG_FACTORY_SLEEPI_REF_UA 0
#endif

#ifndef CONFIG_FACTORY_BLE_SCAN_WINDOW_MS
#define CONFIG_FACTORY_BLE_SCAN_WINDOW_MS 1500
#endif

#ifndef CONFIG_FACTORY_BLE_RSSI_REF_DBM
#define CONFIG_FACTORY_BLE_RSSI_REF_DBM -70
#endif

#ifndef CONFIG_FACTORY_DMIC_SAMPLE_RATE_HZ
#define CONFIG_FACTORY_DMIC_SAMPLE_RATE_HZ 16000
#endif

#ifndef CONFIG_FACTORY_DMIC_BLOCK_SIZE
#define CONFIG_FACTORY_DMIC_BLOCK_SIZE 640
#endif

#ifndef CONFIG_FACTORY_DMIC_READ_TIMEOUT_MS
#define CONFIG_FACTORY_DMIC_READ_TIMEOUT_MS 1000
#endif

#define FACTORY_ADC_CHANNEL_COUNT 8
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define ADC_NODE ZEPHYR_USER_NODE

struct at_ctx {
	const struct device *uart;
	const struct device *regulator_parent;
	struct factory_persist *persist;
};

static struct at_ctx g_ctx;

static const struct adc_dt_spec g_adc_channels[] = {
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 1),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 2),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 3),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 4),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 5),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 6),
	ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 7),
};

static bool g_adc_initialized;

static const gpio_pin_t g_gpio_loop_out_pins[] = { 0, 1, 2 };
static const gpio_pin_t g_gpio_loop_in_pins[] = { 7, 6, 5 };
static const gpio_pin_t g_nfc_loop_out_pins[] = { 3, 4 };
static const gpio_pin_t g_nfc_loop_in_pins[] = { 1, 2 };

static const struct gpio_dt_spec g_sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback g_sw0_cb_data;
static bool g_keywake_ready;
static uint32_t g_keywake_irq_count;
static bool g_sleepi_system_off_pending;

#if DT_NODE_EXISTS(DT_ALIAS(imu0))
static const struct device *const g_imu_dev = DEVICE_DT_GET(DT_ALIAS(imu0));
#endif

#if DT_NODE_EXISTS(DT_ALIAS(dmic20))
static const struct device *const g_dmic_dev = DEVICE_DT_GET(DT_ALIAS(dmic20));
K_MEM_SLAB_DEFINE_STATIC(g_dmic_mem_slab, CONFIG_FACTORY_DMIC_BLOCK_SIZE, 4, 4);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(power_en))
static const struct device *const g_power_en_dev =
	DEVICE_DT_GET(DT_NODELABEL(power_en));
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(imu_vdd))
static const struct device *const g_imu_vdd_dev =
	DEVICE_DT_GET(DT_NODELABEL(imu_vdd));
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(dmic_vdd))
static const struct device *const g_dmic_vdd_dev =
	DEVICE_DT_GET(DT_NODELABEL(dmic_vdd));
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(pmic_charger))
static const struct device *const g_pmic_charger_dev =
	DEVICE_DT_GET(DT_NODELABEL(pmic_charger));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(py25q64), okay)
static const struct device *const g_ext_flash_dev =
	DEVICE_DT_GET(DT_NODELABEL(py25q64));
static const struct device *const g_ext_flash_bus_dev =
	DEVICE_DT_GET(DT_BUS(DT_NODELABEL(py25q64)));
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(gpio2))
static const struct device *const g_gpio2_dev =
	DEVICE_DT_GET(DT_NODELABEL(gpio2));
#endif

#if defined(CONFIG_BT)
struct ble_scan_stats {
	bool have_first;
	bool have_best;
	uint32_t adv_count;
	int8_t first_rssi;
	int8_t best_rssi;
	bt_addr_le_t first_addr;
	bt_addr_le_t best_addr;
};

static bool g_ble_initialized;
static struct ble_scan_stats g_ble_scan_stats;
#endif

static void uart_send_str(const char *s)
{
	while (*s != '\0') {
		uart_poll_out(g_ctx.uart, (uint8_t)*s++);
	}
}

static void uart_send_line(const char *s)
{
	uart_send_str(s);
	uart_send_str("\r\n");
}

static void uart_send_u32(uint32_t value)
{
	char buf[11];
	int i = 0;

	if (value == 0u) {
		uart_poll_out(g_ctx.uart, '0');
		return;
	}

	while (value > 0u && i < (int)sizeof(buf)) {
		buf[i++] = (char)('0' + (value % 10u));
		value /= 10u;
	}

	while (i > 0) {
		uart_poll_out(g_ctx.uart, (uint8_t)buf[--i]);
	}
}

static void uart_send_s32(int32_t value)
{
	if (value < 0) {
		uart_poll_out(g_ctx.uart, '-');
		uart_send_u32((uint32_t)(-(int64_t)value));
		return;
	}

	uart_send_u32((uint32_t)value);
}

static void emit_testdata(const char *state, const char *item,
			  const char *value, const char *unit,
			  const char *raw, const char *meta)
{
	uart_send_str("+TESTDATA:");
	uart_send_str(state);
	uart_send_str(",ITEM=");
	uart_send_str(item);
	uart_send_str(",VALUE=");
	uart_send_str(value);
	uart_send_str(",UNIT=");
	uart_send_str(unit);
	uart_send_str(",RAW=");
	uart_send_str(raw);
	uart_send_str(",META=");
	uart_send_line(meta);
}

static void emit_testdata_cfg_u32(const char *item, uint32_t value,
				  const char *unit, const char *meta)
{
	uart_send_str("+TESTDATA:CFG,ITEM=");
	uart_send_str(item);
	uart_send_str(",VALUE=");
	uart_send_u32(value);
	uart_send_str(",UNIT=");
	uart_send_str(unit);
	uart_send_str(",RAW=kconfig,META=");
	uart_send_line(meta);
}

static void emit_testdata_cfg_s32(const char *item, int32_t value,
				  const char *unit, const char *meta)
{
	uart_send_str("+TESTDATA:CFG,ITEM=");
	uart_send_str(item);
	uart_send_str(",VALUE=");
	uart_send_s32(value);
	uart_send_str(",UNIT=");
	uart_send_str(unit);
	uart_send_str(",RAW=kconfig,META=");
	uart_send_line(meta);
}

static void emit_testdata_adc(const char *state, const char *item, int32_t mv,
			      int16_t raw, uint32_t ref_mv, uint8_t channel)
{
	uart_send_str("+TESTDATA:");
	uart_send_str(state);
	uart_send_str(",ITEM=");
	uart_send_str(item);
	uart_send_str(",VALUE=");
	uart_send_s32(mv);
	uart_send_str(",UNIT=mV,RAW=");
	uart_send_s32((int32_t)raw);
	uart_send_str(",META=ch:");
	uart_send_u32(channel);
	uart_send_str(";ref_mv:");
	uart_send_u32(ref_mv);
	uart_send_line("");
}

static void emit_testdata_bits(const char *state, const char *item,
			       uint32_t hit_count, uint32_t raw_mask,
			       const char *meta)
{
	uart_send_str("+TESTDATA:");
	uart_send_str(state);
	uart_send_str(",ITEM=");
	uart_send_str(item);
	uart_send_str(",VALUE=");
	uart_send_u32(hit_count);
	uart_send_str(",UNIT=bits,RAW=");
	uart_send_u32(raw_mask);
	uart_send_str(",META=");
	uart_send_line(meta);
}

static const char *error_reason_from_rc(int rc)
{
	switch (rc) {
	case 0:
		return "OK";
	case -ENOSYS:
		return "NOT_IMPLEMENTED";
	case -EINVAL:
		return "INVALID_PARAM";
	case -ENODEV:
		return "HW_NOT_READY";
	case -EACCES:
		return "PRECONDITION_NOT_MET";
	case -EIO:
		return "STORAGE_IO_FAIL";
	case -ENOENT:
		return "UNKNOWN_COMMAND";
	default:
		return "EXECUTION_FAILED";
	}
}

static void emit_final_status(int rc)
{
	if (rc == 0) {
		uart_send_line("OK");
		return;
	}

	uart_send_str("ERROR:");
	uart_send_line(error_reason_from_rc(rc));
}

static int at_adc_ensure_ready(void)
{
	int rc;

	if (g_adc_initialized) {
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(g_adc_channels); ++i) {
		if (!adc_is_ready_dt(&g_adc_channels[i])) {
			return -ENODEV;
		}

		rc = adc_channel_setup_dt(&g_adc_channels[i]);
		if (rc != 0) {
			return -ENODEV;
		}
	}

	g_adc_initialized = true;
	return 0;
}

static int at_adc_sample_mv(uint8_t channel_idx, int32_t *mv, int16_t *raw)
{
	struct adc_sequence sequence = {
		.buffer = raw,
		.buffer_size = sizeof(*raw),
	};
	int32_t mv_tmp;
	int rc;

	if (mv == NULL || raw == NULL || channel_idx >= ARRAY_SIZE(g_adc_channels)) {
		return -EINVAL;
	}

	rc = at_adc_ensure_ready();
	if (rc != 0) {
		return rc;
	}

	(void)adc_sequence_init_dt(&g_adc_channels[channel_idx], &sequence);
	rc = adc_read_dt(&g_adc_channels[channel_idx], &sequence);
	if (rc != 0) {
		return -ENODEV;
	}

	mv_tmp = *raw;
	rc = adc_raw_to_millivolts_dt(&g_adc_channels[channel_idx], &mv_tmp);
	if (rc != 0) {
		return -ENODEV;
	}

	*mv = mv_tmp;
	return 0;
}

static int at_handle_vbus(void)
{
	int32_t mv;
	int16_t raw;
	int rc;

	rc = at_adc_sample_mv((uint8_t)CONFIG_FACTORY_ADC_VBUS_CHANNEL, &mv, &raw);
	if (rc != 0) {
		emit_testdata("STATE1", "VBUS", "0", "mV", "adc_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_adc("STATE1", "VBUS", mv, raw,
			 CONFIG_FACTORY_VBUS_USB_MIN_MV,
			 (uint8_t)CONFIG_FACTORY_ADC_VBUS_CHANNEL);
	return 0;
}

static int at_handle_v3p3(void)
{
	int32_t mv;
	int16_t raw;
	int rc;

	rc = at_adc_sample_mv((uint8_t)CONFIG_FACTORY_ADC_3V3_CHANNEL, &mv, &raw);
	if (rc != 0) {
		emit_testdata("STATE1", "3V3", "0", "mV", "adc_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_adc("STATE1", "3V3", mv, raw,
			 CONFIG_FACTORY_3V3_MIN_MV,
			 (uint8_t)CONFIG_FACTORY_ADC_3V3_CHANNEL);
	return 0;
}

static int at_handle_uartloop(void)
{
	if (g_ctx.uart == NULL || !device_is_ready(g_ctx.uart)) {
		emit_testdata("STATE1", "UARTLOOP", "0", "bool", "uart21_not_ready",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	emit_testdata("STATE1", "UARTLOOP", "1", "bool", "uart21_ready",
		      "baud:115200");
	return 0;
}

static int at_run_gpio_pairs(const struct device *out_port,
			     const gpio_pin_t *out_pins,
			     const struct device *in_port,
			     const gpio_pin_t *in_pins,
			     size_t count, uint32_t *raw_mask)
{
	if (out_port == NULL || in_port == NULL || out_pins == NULL ||
	    in_pins == NULL || raw_mask == NULL || count == 0) {
		return -EINVAL;
	}

	if (!device_is_ready(out_port) || !device_is_ready(in_port)) {
		return -ENODEV;
	}

	*raw_mask = 0;

	for (size_t i = 0; i < count; ++i) {
		if (gpio_pin_configure(out_port, out_pins[i], GPIO_OUTPUT_INACTIVE) != 0) {
			return -ENODEV;
		}

		if (gpio_pin_configure(in_port, in_pins[i], GPIO_INPUT) != 0) {
			return -ENODEV;
		}
	}

	k_usleep(100);

	for (size_t i = 0; i < count; ++i) {
		int sampled;

		(void)gpio_pin_set(out_port, out_pins[i], 1);
		k_usleep(200);
		sampled = gpio_pin_get(in_port, in_pins[i]);
		(void)gpio_pin_set(out_port, out_pins[i], 0);

		if (sampled < 0) {
			return -ENODEV;
		}

		if (sampled > 0) {
			*raw_mask |= BIT(i);
		}
	}

	return 0;
}

static void at_keywake_irq_cb(const struct device *port,
			      struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	g_keywake_irq_count++;
}

static int at_keywake_ensure_ready(void)
{
	int rc;

	if (g_keywake_ready) {
		return 0;
	}

	if (!gpio_is_ready_dt(&g_sw0)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&g_sw0, GPIO_INPUT);
	if (rc != 0) {
		return -ENODEV;
	}

	gpio_init_callback(&g_sw0_cb_data, at_keywake_irq_cb, BIT(g_sw0.pin));
	rc = gpio_add_callback(g_sw0.port, &g_sw0_cb_data);
	if (rc != 0) {
		return -ENODEV;
	}

	rc = gpio_pin_interrupt_configure_dt(&g_sw0, GPIO_INT_EDGE_BOTH);
	if (rc != 0) {
		(void)gpio_remove_callback(g_sw0.port, &g_sw0_cb_data);
		return -ENODEV;
	}

	g_keywake_ready = true;
	return 0;
}

static int at_keywake_configure_system_off_wakeup(void)
{
	int rc;

	if (!gpio_is_ready_dt(&g_sw0)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&g_sw0, GPIO_INPUT);
	if (rc != 0) {
		return -ENODEV;
	}

	rc = gpio_pin_interrupt_configure_dt(&g_sw0, GPIO_INT_LEVEL_ACTIVE);
	if (rc != 0) {
		return -ENODEV;
	}

	return 0;
}

static int at_sleepi_configure_spi_pins(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(gpio2))
	int rc;

	if (!device_is_ready(g_gpio2_dev)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure(g_gpio2_dev, 5, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		return rc;
	}

	rc = gpio_pin_configure(g_gpio2_dev, 0, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		return rc;
	}

	rc = gpio_pin_configure(g_gpio2_dev, 3, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		return rc;
	}

	rc = gpio_pin_configure(g_gpio2_dev, 1, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		return rc;
	}

	rc = gpio_pin_configure(g_gpio2_dev, 2, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		return rc;
	}

	rc = gpio_pin_configure(g_gpio2_dev, 4, GPIO_INPUT | GPIO_PULL_DOWN);
	if (rc != 0) {
		return rc;
	}

	return 0;
#else
	return -ENODEV;
#endif
}

static int at_sleepi_suspend_external_flash(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(py25q64), okay) && defined(CONFIG_PM_DEVICE)
	int rc;

	if (!device_is_ready(g_ext_flash_dev)) {
		return -ENODEV;
	}

	rc = pm_device_action_run(g_ext_flash_dev, PM_DEVICE_ACTION_SUSPEND);
	if (rc != 0) {
		return rc;
	}

	if (device_is_ready(g_ext_flash_bus_dev)) {
		rc = pm_device_action_run(g_ext_flash_bus_dev,
					 PM_DEVICE_ACTION_SUSPEND);
		if (rc != 0) {
			return rc;
		}
	}

	return at_sleepi_configure_spi_pins();
#else
	return -ENODEV;
#endif
}

static const char *at_keywake_reset_source(uint32_t flags, uint32_t reset_cause)
{
	if ((flags & FACTORY_PERSIST_FLAG_KEYWAKE_SW0) != 0U) {
		return "gpio";
	}

	if (reset_cause != 0U) {
		return "other";
	}

	return "none";
}

static int at_enable_imu_power(void)
{
	int ret;

#if DT_NODE_EXISTS(DT_NODELABEL(power_en))
	if (!device_is_ready(g_power_en_dev)) {
		return -ENODEV;
	}
	ret = regulator_enable(g_power_en_dev);
	if (ret < 0 && ret != -EALREADY) {
		return -ENODEV;
	}
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(imu_vdd))
	if (!device_is_ready(g_imu_vdd_dev)) {
		return -ENODEV;
	}
	ret = regulator_enable(g_imu_vdd_dev);
	if (ret < 0 && ret != -EALREADY) {
		return -ENODEV;
	}
#endif

	k_sleep(K_MSEC(20));
	return 0;
}

static int at_ensure_imu_ready(void)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	int ret;

	ret = at_enable_imu_power();
	if (ret != 0) {
		return ret;
	}

	if (!device_is_ready(g_imu_dev)) {
		ret = device_init(g_imu_dev);
		if (ret < 0 && ret != -EALREADY) {
			return ret;
		}
	}

	if (!device_is_ready(g_imu_dev)) {
		return -ENODEV;
	}

	return 0;
#else
	return -ENODEV;
#endif
}

static int at_fetch_imu_sample(struct sensor_value *accel_x,
			       struct sensor_value *accel_y,
			       struct sensor_value *accel_z,
			       struct sensor_value *gyro_x,
			       struct sensor_value *gyro_y,
			       struct sensor_value *gyro_z)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	struct sensor_value odr_attr = {
		.val1 = 26,
		.val2 = 0,
	};
	int ret;

	ret = sensor_attr_set(g_imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_attr_set(g_imu_dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		return ret;
	}

	/* Wait one ODR period so the first single-shot read is not a stale zero frame. */
	k_sleep(K_MSEC(40));

	ret = sensor_sample_fetch_chan(g_imu_dev, SENSOR_CHAN_ACCEL_XYZ);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(g_imu_dev, SENSOR_CHAN_ACCEL_X, accel_x);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(g_imu_dev, SENSOR_CHAN_ACCEL_Y, accel_y);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(g_imu_dev, SENSOR_CHAN_ACCEL_Z, accel_z);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_sample_fetch_chan(g_imu_dev, SENSOR_CHAN_GYRO_XYZ);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(g_imu_dev, SENSOR_CHAN_GYRO_X, gyro_x);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(g_imu_dev, SENSOR_CHAN_GYRO_Y, gyro_y);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(g_imu_dev, SENSOR_CHAN_GYRO_Z, gyro_z);
	if (ret != 0) {
		return ret;
	}

	return 0;
#else
	ARG_UNUSED(accel_x);
	ARG_UNUSED(accel_y);
	ARG_UNUSED(accel_z);
	ARG_UNUSED(gyro_x);
	ARG_UNUSED(gyro_y);
	ARG_UNUSED(gyro_z);
	return -ENODEV;
#endif
}

static int at_enable_dmic_power(void)
{
	int ret;

#if DT_NODE_EXISTS(DT_NODELABEL(power_en))
	if (!device_is_ready(g_power_en_dev)) {
		return -ENODEV;
	}
	ret = regulator_enable(g_power_en_dev);
	if (ret < 0 && ret != -EALREADY) {
		return -ENODEV;
	}
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(dmic_vdd))
	if (!device_is_ready(g_dmic_vdd_dev)) {
		return -ENODEV;
	}
	ret = regulator_enable(g_dmic_vdd_dev);
	if (ret < 0 && ret != -EALREADY) {
		return -ENODEV;
	}
#endif

	k_sleep(K_MSEC(20));
	return 0;
}

static int at_pmic_charger_fetch(struct sensor_value *avg_current,
				 struct sensor_value *bat_voltage,
				 struct sensor_value *chg_status)
{
#if DT_NODE_EXISTS(DT_NODELABEL(pmic_charger))
	if (!device_is_ready(g_pmic_charger_dev)) {
		return -ENODEV;
	}

	if (sensor_sample_fetch(g_pmic_charger_dev) != 0) {
		return -ENODEV;
	}

	if (avg_current != NULL &&
	    sensor_channel_get(g_pmic_charger_dev, SENSOR_CHAN_GAUGE_AVG_CURRENT,
			       avg_current) != 0) {
		return -ENODEV;
	}

	if (bat_voltage != NULL &&
	    sensor_channel_get(g_pmic_charger_dev, SENSOR_CHAN_GAUGE_VOLTAGE,
			       bat_voltage) != 0) {
		return -ENODEV;
	}

	if (chg_status != NULL &&
	    sensor_channel_get(g_pmic_charger_dev,
			       SENSOR_CHAN_NPM13XX_CHARGER_STATUS,
			       chg_status) != 0) {
		chg_status->val1 = 0;
		chg_status->val2 = 0;
	}

	return 0;
#else
	ARG_UNUSED(avg_current);
	ARG_UNUSED(bat_voltage);
	ARG_UNUSED(chg_status);
	return -ENODEV;
#endif
}

static int at_handle_chgcur(void)
{
	struct sensor_value avg_current;
	struct sensor_value chg_status;
	int64_t current_ua;
	int32_t current_ma;
	int rc;

	rc = at_pmic_charger_fetch(&avg_current, NULL, &chg_status);
	if (rc != 0) {
		emit_testdata("STATE2", "CHGCUR", "0", "mA", "charger_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	current_ua = sensor_value_to_micro(&avg_current);
	current_ma = (int32_t)(current_ua / 1000);

	uart_send_str("+TESTDATA:STATE2,ITEM=CHGCUR,VALUE=");
	uart_send_s32(current_ma);
	uart_send_str(",UNIT=mA,RAW=");
	uart_send_s32((int32_t)current_ua);
	uart_send_str(",META=src:pmic_charger;status:");
	uart_send_s32(chg_status.val1);
	uart_send_str(";ref_ma:");
	uart_send_u32(CONFIG_FACTORY_CHGCUR_REF_MA);
	uart_send_line(";sensor:gauge_avg_current");
	return 0;
}

static int at_handle_batv(void)
{
	struct sensor_value bat_voltage;
	int32_t bat_mv;
	int16_t bat_raw;
	int rc;

	rc = at_pmic_charger_fetch(NULL, &bat_voltage, NULL);
	if (rc == 0) {
		int64_t bat_uv = sensor_value_to_micro(&bat_voltage);

		bat_mv = (int32_t)(bat_uv / 1000);
		uart_send_str("+TESTDATA:STATE2,ITEM=BATV,VALUE=");
		uart_send_s32(bat_mv);
		uart_send_str(",UNIT=mV,RAW=");
		uart_send_s32((int32_t)bat_uv);
		uart_send_str(",META=src:pmic_charger;ref_delta_mv:");
		uart_send_u32(CONFIG_FACTORY_BATV_DELTA_MAX_MV);
		uart_send_line(";sensor:gauge_voltage");
		return 0;
	}

	rc = at_adc_sample_mv((uint8_t)CONFIG_FACTORY_ADC_BATV_CHANNEL, &bat_mv, &bat_raw);
	if (rc != 0) {
		emit_testdata("STATE2", "BATV", "0", "mV", "batv_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_adc("STATE2", "BATV", bat_mv, bat_raw,
			  CONFIG_FACTORY_BATV_DELTA_MAX_MV,
			  (uint8_t)CONFIG_FACTORY_ADC_BATV_CHANNEL);
	return 0;
}

static int at_ble_ensure_ready(void)
{
#if defined(CONFIG_BT)
	int rc;

	if (g_ble_initialized) {
		return 0;
	}

	rc = bt_enable(NULL);
	if (rc < 0 && rc != -EALREADY) {
		return -ENODEV;
	}

	g_ble_initialized = true;
	return 0;
#else
	return -ENODEV;
#endif
}

#if defined(CONFIG_BT)
static bool at_ble_adv_type_supported(uint8_t type)
{
	return (type == BT_GAP_ADV_TYPE_ADV_IND) ||
	       (type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) ||
	       (type == BT_GAP_ADV_TYPE_ADV_SCAN_IND) ||
	       (type == BT_GAP_ADV_TYPE_SCAN_RSP);
}

static void at_ble_scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			   struct net_buf_simple *ad)
{
	ARG_UNUSED(ad);

	if (!at_ble_adv_type_supported(type)) {
		return;
	}

	g_ble_scan_stats.adv_count++;

	if (!g_ble_scan_stats.have_first) {
		g_ble_scan_stats.have_first = true;
		g_ble_scan_stats.first_rssi = rssi;
		bt_addr_le_copy(&g_ble_scan_stats.first_addr, addr);
	}

	if (!g_ble_scan_stats.have_best || rssi > g_ble_scan_stats.best_rssi) {
		g_ble_scan_stats.have_best = true;
		g_ble_scan_stats.best_rssi = rssi;
		bt_addr_le_copy(&g_ble_scan_stats.best_addr, addr);
	}
}
#endif

static int at_handle_blescan(void)
{
#if defined(CONFIG_BT)
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	char first_addr[BT_ADDR_LE_STR_LEN] = "none";
	char best_addr[BT_ADDR_LE_STR_LEN] = "none";
	int rc;
	int stop_rc;
	int32_t best_rssi = -127;

	rc = at_ble_ensure_ready();
	if (rc != 0) {
		emit_testdata("STATE3", "BLESCAN", "0", "adv", "bt_init_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	memset(&g_ble_scan_stats, 0, sizeof(g_ble_scan_stats));
	rc = bt_le_scan_start(&scan_param, at_ble_scan_cb);
	if (rc != 0 && rc != -EALREADY) {
		emit_testdata("STATE3", "BLESCAN", "0", "adv", "scan_start_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	k_msleep(CONFIG_FACTORY_BLE_SCAN_WINDOW_MS);
	stop_rc = bt_le_scan_stop();
	if (stop_rc != 0 && stop_rc != -EALREADY) {
		emit_testdata("STATE3", "BLESCAN", "0", "adv", "scan_stop_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	if (g_ble_scan_stats.have_first) {
		bt_addr_le_to_str(&g_ble_scan_stats.first_addr, first_addr,
				  sizeof(first_addr));
	}
	if (g_ble_scan_stats.have_best) {
		best_rssi = g_ble_scan_stats.best_rssi;
		bt_addr_le_to_str(&g_ble_scan_stats.best_addr, best_addr,
				  sizeof(best_addr));
	}

	uart_send_str("+TESTDATA:STATE3,ITEM=BLESCAN,VALUE=");
	uart_send_u32(g_ble_scan_stats.adv_count);
	uart_send_str(",UNIT=adv,RAW=");
	uart_send_s32(best_rssi);
	uart_send_str(",META=first:");
	uart_send_str(first_addr);
	uart_send_str(";best:");
	uart_send_str(best_addr);
	uart_send_str(";window_ms:");
	uart_send_u32(CONFIG_FACTORY_BLE_SCAN_WINDOW_MS);
	uart_send_str(";ref_rssi_dbm:");
	uart_send_s32(CONFIG_FACTORY_BLE_RSSI_REF_DBM);
	uart_send_line(";mode:active_scan");
	return 0;
#else
	emit_testdata("STATE3", "BLESCAN", "0", "adv", "bt_not_enabled",
		      "err:HW_NOT_READY");
	return -ENODEV;
#endif
}

static int at_handle_gpioloop(void)
{
	const struct device *const gpio3 =
		DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio3));
	uint32_t raw_mask;
	int rc;

	rc = at_run_gpio_pairs(gpio3, g_gpio_loop_out_pins,
			       gpio3, g_gpio_loop_in_pins,
			       ARRAY_SIZE(g_gpio_loop_out_pins), &raw_mask);
	if (rc != 0) {
		emit_testdata("STATE3", "GPIOLOOP", "0", "bits", "gpio_loop_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_bits("STATE3", "GPIOLOOP", POPCOUNT(raw_mask), raw_mask,
			   "map:D11->D18;D12->D17;D13->D16");
	return 0;
}

static int at_handle_nfcloop(void)
{
	const struct device *const gpio3 =
		DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio3));
	const struct device *const gpio1 =
		DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio1));
	uint32_t raw_mask;
	int rc;

	rc = at_run_gpio_pairs(gpio3, g_nfc_loop_out_pins,
			       gpio1, g_nfc_loop_in_pins,
			       ARRAY_SIZE(g_nfc_loop_out_pins), &raw_mask);
	if (rc != 0) {
		emit_testdata("STATE3", "NFCLOOP", "0", "bits", "nfc_loop_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_bits("STATE3", "NFCLOOP", POPCOUNT(raw_mask), raw_mask,
			   "map:D14->NFC1;D15->NFC2");
	return 0;
}

static int at_handle_imu6d(void)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	struct sensor_value accel_x, accel_y, accel_z;
	struct sensor_value gyro_x, gyro_y, gyro_z;
	int rc;

	rc = at_ensure_imu_ready();
	if (rc != 0) {
		emit_testdata("STATE4", "IMU6D", "0", "sample", "imu_not_ready",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	rc = at_fetch_imu_sample(&accel_x, &accel_y, &accel_z,
				 &gyro_x, &gyro_y, &gyro_z);
	if (rc != 0) {
		emit_testdata("STATE4", "IMU6D", "0", "sample", "sample_fetch_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	uart_send_str("+TESTDATA:STATE4,ITEM=IMU6D,VALUE=1,UNIT=sample,RAW=ax_u:");
	uart_send_s32(sensor_value_to_micro(&accel_x));
	uart_send_str(";ay_u:");
	uart_send_s32(sensor_value_to_micro(&accel_y));
	uart_send_str(";az_u:");
	uart_send_s32(sensor_value_to_micro(&accel_z));
	uart_send_str(";gx_u:");
	uart_send_s32(sensor_value_to_micro(&gyro_x));
	uart_send_str(";gy_u:");
	uart_send_s32(sensor_value_to_micro(&gyro_y));
	uart_send_str(";gz_u:");
	uart_send_s32(sensor_value_to_micro(&gyro_z));
	uart_send_line(",META=mode:single_fetch;src:imu0");
	return 0;
#else
	emit_testdata("STATE4", "IMU6D", "0", "sample", "imu_alias_missing",
		      "err:HW_NOT_READY");
	return -ENODEV;
#endif
}

static int at_handle_micamp(void)
{
#if DT_NODE_EXISTS(DT_ALIAS(dmic20))
	struct pcm_stream_cfg stream_cfg = {
		.pcm_rate = CONFIG_FACTORY_DMIC_SAMPLE_RATE_HZ,
		.pcm_width = 16,
		.block_size = CONFIG_FACTORY_DMIC_BLOCK_SIZE,
		.mem_slab = &g_dmic_mem_slab,
	};
	struct dmic_cfg dmic_cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream_cfg,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = 1,
		},
	};
	void *buffer = NULL;
	void *discard_buffer = NULL;
	uint32_t size = 0;
	uint32_t samples;
	int64_t sum_abs = 0;
	int32_t peak = 0;
	int32_t avg;
	int rc;

	rc = at_enable_dmic_power();
	if (rc != 0) {
		emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_power_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	if (!device_is_ready(g_dmic_dev)) {
		emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_not_ready",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	dmic_cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

	rc = dmic_configure(g_dmic_dev, &dmic_cfg);
	if (rc != 0) {
		emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_config_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	rc = dmic_trigger(g_dmic_dev, DMIC_TRIGGER_START);
	if (rc != 0) {
		emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_start_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	/* Discard the first block to let the DMIC pipeline settle, matching the working sample. */
	rc = dmic_read(g_dmic_dev, 0, &discard_buffer, &size,
		       CONFIG_FACTORY_DMIC_READ_TIMEOUT_MS);
	if (rc == 0 && discard_buffer != NULL) {
		(void)k_mem_slab_free(&g_dmic_mem_slab, discard_buffer);
		discard_buffer = NULL;
	}

	rc = dmic_read(g_dmic_dev, 0, &buffer, &size, CONFIG_FACTORY_DMIC_READ_TIMEOUT_MS);
	(void)dmic_trigger(g_dmic_dev, DMIC_TRIGGER_STOP);
	if (rc != 0 || buffer == NULL || size < sizeof(int16_t)) {
		emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_read_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	samples = size / sizeof(int16_t);
	for (uint32_t i = 0; i < samples; ++i) {
		int32_t v = ((int16_t *)buffer)[i];
		int32_t abs_v = (v < 0) ? -v : v;

		if (abs_v > peak) {
			peak = abs_v;
		}
		sum_abs += abs_v;
	}
	avg = (samples > 0) ? (int32_t)(sum_abs / samples) : 0;
	(void)k_mem_slab_free(&g_dmic_mem_slab, buffer);

	uart_send_str("+TESTDATA:STATE4,ITEM=MICAMP,VALUE=");
	uart_send_s32(peak);
	uart_send_str(",UNIT=abs16,RAW=");
	uart_send_s32(avg);
	uart_send_str(",META=samples:");
	uart_send_u32(samples);
	uart_send_str(";rate:");
	uart_send_u32(CONFIG_FACTORY_DMIC_SAMPLE_RATE_HZ);
	uart_send_line(";mode:single_block;discard:first_block");
	return 0;
#else
	emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_alias_missing",
		      "err:HW_NOT_READY");
	return -ENODEV;
#endif
}

static int at_handle_sleepi(void)
{
	uint32_t old_flags;
	uint32_t old_reset_cause;
	int rc;

	if (g_ctx.persist == NULL) {
		emit_testdata("STATE5", "SLEEPI", "0", "bool", "persist_null",
			      "err:PRECONDITION_NOT_MET");
		return -EACCES;
	}

	rc = at_keywake_configure_system_off_wakeup();
	if (rc != 0) {
		emit_testdata("STATE5", "SLEEPI", "0", "bool", "sw0_wakeup_not_ready",
			      "err:HW_NOT_READY");
		return rc;
	}

	old_flags = g_ctx.persist->reserved[FACTORY_PERSIST_FLAGS_IDX];
	old_reset_cause = g_ctx.persist->reserved[FACTORY_PERSIST_RESET_CAUSE_IDX];

	g_ctx.persist->reserved[FACTORY_PERSIST_FLAGS_IDX] &=
		~(FACTORY_PERSIST_FLAG_KEYWAKE_LATCHED |
		  FACTORY_PERSIST_FLAG_KEYWAKE_SW0);
	g_ctx.persist->reserved[FACTORY_PERSIST_FLAGS_IDX] |=
		FACTORY_PERSIST_FLAG_SLEEPI_ARMED;
	g_ctx.persist->reserved[FACTORY_PERSIST_RESET_CAUSE_IDX] = 0U;

	rc = factory_storage_save(g_ctx.persist);
	if (rc != 0) {
		g_ctx.persist->reserved[FACTORY_PERSIST_FLAGS_IDX] = old_flags;
		g_ctx.persist->reserved[FACTORY_PERSIST_RESET_CAUSE_IDX] =
			old_reset_cause;
		emit_testdata("STATE5", "SLEEPI", "0", "bool", "sleep_state_save_failed",
			      "err:STORAGE_IO_FAIL");
		return -EIO;
	}

	rc = at_sleepi_suspend_external_flash();
	if (rc != 0) {
		g_ctx.persist->reserved[FACTORY_PERSIST_FLAGS_IDX] = old_flags;
		g_ctx.persist->reserved[FACTORY_PERSIST_RESET_CAUSE_IDX] =
			old_reset_cause;
		(void)factory_storage_save(g_ctx.persist);
		emit_testdata("STATE5", "SLEEPI", "0", "bool", "flash_suspend_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	g_sleepi_system_off_pending = true;

	uart_send_str("+TESTDATA:STATE5,ITEM=SLEEPI,VALUE=1,UNIT=bool,RAW=system_off_armed,META=wakeup:sw0;window_ms:");
	uart_send_u32(CONFIG_FACTORY_SLEEPI_WINDOW_MS);
	uart_send_str(";ref_uA:");
	uart_send_u32(CONFIG_FACTORY_SLEEPI_REF_UA);
	uart_send_line(";mode:system_off;flash:dpd");
	return 0;
}

static int at_handle_keywake(void)
{
	int level;
	uint32_t flags = 0U;
	uint32_t wake_count = 0U;
	uint32_t reset_cause = 0U;
	int rc;

	rc = at_keywake_ensure_ready();
	if (rc != 0) {
		emit_testdata("STATE6", "KEYWAKE", "0", "bool", "sw0_not_ready",
			      "err:HW_NOT_READY");
		return rc;
	}

	level = gpio_pin_get_dt(&g_sw0);
	if (level < 0) {
		emit_testdata("STATE6", "KEYWAKE", "0", "bool", "sw0_read_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	if (g_ctx.persist != NULL) {
		flags = g_ctx.persist->reserved[FACTORY_PERSIST_FLAGS_IDX];
		wake_count = g_ctx.persist->reserved[FACTORY_PERSIST_WAKE_COUNT_IDX];
		reset_cause = g_ctx.persist->reserved[FACTORY_PERSIST_RESET_CAUSE_IDX];
	}

	uart_send_str("+TESTDATA:STATE6,ITEM=KEYWAKE,VALUE=");
	uart_send_u32((flags & FACTORY_PERSIST_FLAG_KEYWAKE_LATCHED) != 0U ? 1U : 0U);
	uart_send_str(",UNIT=bool,RAW=");
	uart_send_u32(wake_count);
	uart_send_str(",META=alias:sw0;level:");
	uart_send_u32((uint32_t)level);
	uart_send_str(";irq_count:");
	uart_send_u32(g_keywake_irq_count);
	uart_send_str(";wake_reset:");
	uart_send_str(at_keywake_reset_source(flags, reset_cause));
	uart_send_str(";reset_cause:");
	uart_send_u32(reset_cause);
	uart_send_line("");

	return 0;
}

static int at_handle_flashwrite(void)
{
	struct factory_persist verify;
	uint32_t old_bitmap;
	int rc;

	if (g_ctx.persist == NULL) {
		emit_testdata("STATE7", "FLASHWRITE", "0", "bool", "persist_null",
			      "err:PRECONDITION_NOT_MET");
		return -EACCES;
	}

	old_bitmap = g_ctx.persist->item_bitmap;
	g_ctx.persist->item_bitmap ^= BIT(0);

	rc = factory_storage_save(g_ctx.persist);
	if (rc != 0) {
		g_ctx.persist->item_bitmap = old_bitmap;
		emit_testdata("STATE7", "FLASHWRITE", "0", "bool", "save_failed",
			      "err:STORAGE_IO_FAIL");
		return -EIO;
	}

	rc = factory_storage_load(&verify);
	if (rc != 0 || verify.item_bitmap != g_ctx.persist->item_bitmap) {
		emit_testdata("STATE7", "FLASHWRITE", "0", "bool", "verify_mismatch",
			      "err:STORAGE_IO_FAIL");
		return -EIO;
	}

	emit_testdata("STATE7", "FLASHWRITE", "1", "bool", "write_readback_ok",
		      "op:toggle_item_bitmap_bit0");
	return 0;
}

static int at_handle_shipmode_a(void)
{
	int rc;

	if (g_ctx.regulator_parent == NULL ||
	    !device_is_ready(g_ctx.regulator_parent)) {
		emit_testdata("STATE8A", "SHIPMODE", "0", "bool",
			      "regulator_not_ready", "err:HW_NOT_READY");
		return -ENODEV;
	}

	rc = regulator_parent_ship_mode(g_ctx.regulator_parent);
	if (rc != 0) {
		emit_testdata("STATE8A", "SHIPMODE", "0", "bool", "ship_mode_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	emit_testdata("STATE8A", "SHIPMODE", "1", "bool", "ship_mode_entered",
		      "phase:A");
	return 0;
}

static int at_handle_vbus_b(void)
{
	int32_t mv;
	int16_t raw;
	int rc;

	rc = at_adc_sample_mv((uint8_t)CONFIG_FACTORY_ADC_VBUS_CHANNEL, &mv, &raw);
	if (rc != 0) {
		emit_testdata("STATE8B", "VBUS_B", "0", "mV", "adc_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_adc("STATE8B", "VBUS_B", mv, raw,
			 CONFIG_FACTORY_VBUS_BAT_MAX_MV,
			 (uint8_t)CONFIG_FACTORY_ADC_VBUS_CHANNEL);
	return 0;
}

static int at_handle_v3p3_b(void)
{
	int32_t mv;
	int16_t raw;
	int rc;

	rc = at_adc_sample_mv((uint8_t)CONFIG_FACTORY_ADC_3V3_CHANNEL, &mv, &raw);
	if (rc != 0) {
		emit_testdata("STATE8B", "3V3_B", "0", "mV", "adc_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	emit_testdata_adc("STATE8B", "3V3_B", mv, raw,
			 CONFIG_FACTORY_3V3_MIN_MV,
			 (uint8_t)CONFIG_FACTORY_ADC_3V3_CHANNEL);
	return 0;
}

static int at_handle_shipmode_b(void)
{
	int rc;

	if (g_ctx.regulator_parent == NULL ||
	    !device_is_ready(g_ctx.regulator_parent)) {
		emit_testdata("STATE9B", "SHIPMODE", "0", "bool",
			      "regulator_not_ready", "err:HW_NOT_READY");
		return -ENODEV;
	}

	rc = regulator_parent_ship_mode(g_ctx.regulator_parent);
	if (rc != 0) {
		emit_testdata("STATE9B", "SHIPMODE", "0", "bool", "ship_mode_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	emit_testdata("STATE9B", "SHIPMODE", "1", "bool", "ship_mode_entered",
		      "phase:B");
	return 0;
}

struct state_item {
	int (*fn)(void);
};

static int run_state(const struct state_item *items, size_t count)
{
	int rc = 0;

	for (size_t i = 0; i < count; ++i) {
		int item_rc = items[i].fn();
		if (item_rc != 0 && rc == 0) {
			rc = item_rc;
		}
	}

	return rc;
}

static int at_handle_state_1(void)
{
	static const struct state_item items[] = {
		{ at_handle_vbus },
		{ at_handle_v3p3 },
		{ at_handle_uartloop },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_2(void)
{
	static const struct state_item items[] = {
		{ at_handle_chgcur },
		{ at_handle_batv },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_3(void)
{
	static const struct state_item items[] = {
		{ at_handle_blescan },
		{ at_handle_gpioloop },
		{ at_handle_nfcloop },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_4(void)
{
	static const struct state_item items[] = {
		{ at_handle_imu6d },
		{ at_handle_micamp },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_5(void)
{
	static const struct state_item items[] = {
		{ at_handle_sleepi },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_6(void)
{
	static const struct state_item items[] = {
		{ at_handle_keywake },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_7(void)
{
	static const struct state_item items[] = {
		{ at_handle_flashwrite },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_8a(void)
{
	static const struct state_item items[] = {
		{ at_handle_shipmode_a },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_8b(void)
{
	static const struct state_item items[] = {
		{ at_handle_vbus_b },
		{ at_handle_v3p3_b },
	};
	return run_state(items, ARRAY_SIZE(items));
}

static int at_handle_state_9b(void)
{
	static const struct state_item items[] = {
		{ at_handle_shipmode_b },
	};
	return run_state(items, ARRAY_SIZE(items));
}

struct at_cmd {
	const char *cmd;
	int (*fn)(void);
};

static const struct at_cmd g_item_cmds[] = {
	{ "AT+VBUS", at_handle_vbus },
	{ "AT+V3P3", at_handle_v3p3 },
	{ "AT+UARTLOOP", at_handle_uartloop },
	{ "AT+CHGCUR", at_handle_chgcur },
	{ "AT+BATV", at_handle_batv },
	{ "AT+BLESCAN", at_handle_blescan },
	{ "AT+GPIOLOOP", at_handle_gpioloop },
	{ "AT+NFCLOOP", at_handle_nfcloop },
	{ "AT+IMU6D", at_handle_imu6d },
	{ "AT+MICAMP", at_handle_micamp },
	{ "AT+SLEEPI", at_handle_sleepi },
	{ "AT+KEYWAKE", at_handle_keywake },
	{ "AT+FLASHWRITE", at_handle_flashwrite },
	{ "AT+SHIPMODEA", at_handle_shipmode_a },
	{ "AT+VBUS_B", at_handle_vbus_b },
	{ "AT+V3P3_B", at_handle_v3p3_b },
	{ "AT+SHIPMODEB", at_handle_shipmode_b },
};

static const struct at_cmd g_state_cmds[] = {
	{ "AT+STATE1", at_handle_state_1 },
	{ "AT+STATE2", at_handle_state_2 },
	{ "AT+STATE3", at_handle_state_3 },
	{ "AT+STATE4", at_handle_state_4 },
	{ "AT+STATE5", at_handle_state_5 },
	{ "AT+STATE6", at_handle_state_6 },
	{ "AT+STATE7", at_handle_state_7 },
	{ "AT+STATE8A", at_handle_state_8a },
	{ "AT+STATE8B", at_handle_state_8b },
	{ "AT+STATE9B", at_handle_state_9b },
};

static int at_handle_help(void)
{
	uart_send_line("+CMDS:AT,AT+HELP,AT+FLASH?,AT+FLASH=<0|1|2>,AT+THRESH?");
	uart_send_line("+CMDS:AT+STATE1..AT+STATE7,AT+STATE8A,AT+STATE8B,AT+STATE9B");
	uart_send_line("+CMDS:AT+VBUS,AT+V3P3,AT+UARTLOOP,AT+CHGCUR,AT+BATV");
	uart_send_line("+CMDS:AT+BLESCAN,AT+GPIOLOOP,AT+NFCLOOP,AT+IMU6D,AT+MICAMP");
	uart_send_line("+CMDS:AT+SLEEPI,AT+KEYWAKE,AT+FLASHWRITE,AT+SHIPMODEA,AT+VBUS_B,AT+V3P3_B,AT+SHIPMODEB");
	return 0;
}

static int at_handle_flash_get(void)
{
	if (g_ctx.persist == NULL) {
		emit_testdata("SYS", "FLASH_FLAG", "0", "raw", "persist_null",
			      "err:PRECONDITION_NOT_MET");
		return -EACCES;
	}

	uart_send_str("+TESTDATA:SYS,ITEM=FLASH_FLAG,VALUE=");
	uart_send_u32(g_ctx.persist->boot_flag);
	uart_send_line(",UNIT=raw,RAW=boot_flag,META=from_storage");
	return 0;
}

static int at_handle_flash_set(const char *arg)
{
	uint32_t value;
	int rc;

	if (g_ctx.persist == NULL) {
		emit_testdata("SYS", "FLASH_FLAG", "0", "raw", "persist_null",
			      "err:PRECONDITION_NOT_MET");
		return -EACCES;
	}

	if (arg == NULL || strlen(arg) != 1 || arg[0] < '0' || arg[0] > '2') {
		return -EINVAL;
	}

	value = (uint32_t)(arg[0] - '0');
	g_ctx.persist->boot_flag = value;
	rc = factory_storage_save(g_ctx.persist);
	if (rc != 0) {
		emit_testdata("SYS", "FLASH_FLAG", "0", "raw", "save_failed",
			      "err:STORAGE_IO_FAIL");
		return -EIO;
	}

	uart_send_str("+TESTDATA:SYS,ITEM=FLASH_FLAG,VALUE=");
	uart_send_u32(value);
	uart_send_line(",UNIT=raw,RAW=set_by_at_flash,META=persisted");
	return 0;
}

static int at_handle_thresh_get(void)
{
	emit_testdata_cfg_u32("VBUS_USB_MIN_MV", CONFIG_FACTORY_VBUS_USB_MIN_MV,
			     "mV", "state:STATE1,item:VBUS");
	emit_testdata_cfg_u32("3V3_MIN_MV", CONFIG_FACTORY_3V3_MIN_MV,
			     "mV", "state:STATE1|STATE8B,item:3V3");
	emit_testdata_cfg_u32("VBUS_BAT_MAX_MV", CONFIG_FACTORY_VBUS_BAT_MAX_MV,
			     "mV", "state:STATE8B,item:VBUS_B");
	emit_testdata_cfg_u32("ADC_VBUS_CHANNEL", CONFIG_FACTORY_ADC_VBUS_CHANNEL,
			     "idx", "source:zephyr_user.io-channels");
	emit_testdata_cfg_u32("ADC_3V3_CHANNEL", CONFIG_FACTORY_ADC_3V3_CHANNEL,
			     "idx", "source:zephyr_user.io-channels");
	emit_testdata_cfg_u32("ADC_BATV_CHANNEL", CONFIG_FACTORY_ADC_BATV_CHANNEL,
			     "idx", "state:STATE2,item:BATV(fallback)");
	emit_testdata_cfg_u32("CHGCUR_REF_MA", CONFIG_FACTORY_CHGCUR_REF_MA,
			     "mA", "state:STATE2,item:CHGCUR");
	emit_testdata_cfg_u32("BATV_DELTA_MAX_MV", CONFIG_FACTORY_BATV_DELTA_MAX_MV,
			     "mV", "state:STATE2,item:BATV");
	emit_testdata_cfg_u32("SLEEPI_WINDOW_MS", CONFIG_FACTORY_SLEEPI_WINDOW_MS,
			     "ms", "state:STATE5,item:SLEEPI");
	emit_testdata_cfg_u32("SLEEPI_REF_UA", CONFIG_FACTORY_SLEEPI_REF_UA,
			     "uA", "state:STATE5,item:SLEEPI");
	emit_testdata_cfg_u32("BLE_SCAN_WINDOW_MS", CONFIG_FACTORY_BLE_SCAN_WINDOW_MS,
			     "ms", "state:STATE3,item:BLESCAN");
	emit_testdata_cfg_s32("BLE_RSSI_REF_DBM", CONFIG_FACTORY_BLE_RSSI_REF_DBM,
			     "dBm", "state:STATE3,item:BLESCAN");
	return 0;
}

static bool at_cmd_equals(const char *trimmed, size_t trimmed_len,
			 const char *cmd)
{
	size_t cmd_len = strlen(cmd);

	return trimmed_len == cmd_len && strncmp(trimmed, cmd, cmd_len) == 0;
}

static int dispatch_from_table(const char *trimmed, size_t trimmed_len,
			       const struct at_cmd *table, size_t count,
			       bool track_state)
{
	for (size_t i = 0; i < count; ++i) {
		if (at_cmd_equals(trimmed, trimmed_len, table[i].cmd)) {
			int rc = table[i].fn();

			if (rc == 0 && track_state && g_ctx.persist != NULL) {
				g_ctx.persist->state_bitmap |= BIT(i);
				if (factory_storage_save(g_ctx.persist) != 0) {
					rc = -EIO;
				}
			}

			return rc;
		}
	}

	return -ENOENT;
}

void at_handler_init(const struct device *uart_dev,
		     const struct device *regulator_parent,
		     struct factory_persist *persist)
{
	g_ctx.uart = uart_dev;
	g_ctx.regulator_parent = regulator_parent;
	g_ctx.persist = persist;
	g_adc_initialized = false;
	g_keywake_ready = false;
	g_keywake_irq_count = 0;
	g_sleepi_system_off_pending = false;
}

void at_handler_run_deferred_action(void)
{
	if (!g_sleepi_system_off_pending) {
		return;
	}

	g_sleepi_system_off_pending = false;

#if defined(CONFIG_HWINFO)
	(void)hwinfo_clear_reset_cause();
#endif

	/* Allow the trailing "OK" response to drain before UART21 is suspended. */
	k_msleep(20);

#if defined(CONFIG_PM_DEVICE)
	if (g_ctx.uart != NULL && device_is_ready(g_ctx.uart)) {
		(void)pm_device_action_run(g_ctx.uart, PM_DEVICE_ACTION_SUSPEND);
	}
#endif

	k_msleep(20);
	sys_poweroff();
}

void at_handler_process_line(const char *line, size_t len)
{
	const char *trimmed = line;
	size_t trimmed_len = len;
	int rc;

	if (line == NULL || len == 0) {
		return;
	}

	while (trimmed_len > 0 && isspace((unsigned char)trimmed[trimmed_len - 1])) {
		trimmed_len--;
	}

	while (trimmed_len > 0 && isspace((unsigned char)*trimmed)) {
		trimmed++;
		trimmed_len--;
	}

	if (trimmed_len == 0) {
		return;
	}

	if (at_cmd_equals(trimmed, trimmed_len, "AT")) {
		uart_send_line("OK");
		return;
	}

	if (at_cmd_equals(trimmed, trimmed_len, "AT+HELP")) {
		rc = at_handle_help();
		emit_final_status(rc);
		return;
	}

	if (at_cmd_equals(trimmed, trimmed_len, "AT+FLASH?")) {
		rc = at_handle_flash_get();
		emit_final_status(rc);
		return;
	}

	if (at_cmd_equals(trimmed, trimmed_len, "AT+THRESH?")) {
		rc = at_handle_thresh_get();
		emit_final_status(rc);
		return;
	}

	if (trimmed_len >= strlen("AT+FLASH=") &&
	    strncmp(trimmed, "AT+FLASH=", strlen("AT+FLASH=")) == 0) {
		rc = at_handle_flash_set(trimmed + strlen("AT+FLASH="));
		emit_final_status(rc);
		return;
	}

	rc = dispatch_from_table(trimmed, trimmed_len,
				 g_item_cmds, ARRAY_SIZE(g_item_cmds), false);
	if (rc != -ENOENT) {
		emit_final_status(rc);
		return;
	}

	rc = dispatch_from_table(trimmed, trimmed_len,
				 g_state_cmds, ARRAY_SIZE(g_state_cmds), true);
	if (rc != -ENOENT) {
		emit_final_status(rc);
		return;
	}

	emit_final_status(-ENOENT);
}
