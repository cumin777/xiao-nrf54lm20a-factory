#include "at_handler.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
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
#include <zephyr/sys/atomic.h>
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

#ifndef CONFIG_FACTORY_IMU_TRACE
#define CONFIG_FACTORY_IMU_TRACE 0
#endif

#define FACTORY_ADC_CHANNEL_COUNT 8
#define FACTORY_CMD_PARSE_BUF_SIZE 160
#define FACTORY_TEXT_CMD_MAX_TOKENS 6
#define FACTORY_UART20_BUF_SIZE 32
#define FACTORY_IMU_FIRST_SAMPLE_WAIT_MS 200
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define ADC_NODE ZEPHYR_USER_NODE

struct at_ctx {
	const struct device *uart;
	const struct device *uart_primary;
	const struct device *regulator_parent;
	struct factory_persist *persist;
};

struct mic_capture_stats {
	int32_t min_sample;
	int32_t max_sample;
	int32_t peak_abs;
	int32_t avg_abs;
	uint32_t max_consecutive;
	uint32_t sample_count;
};

struct gpio_text_endpoint {
	uint8_t group;
	gpio_pin_t pin;
};

struct gpio_text_pair_desc {
	struct gpio_text_endpoint endpoints[2];
};

struct gpio_text_pair_state {
	bool configured;
	uint8_t output_idx;
};

struct imu_sample_cache {
	struct sensor_value accel_x;
	struct sensor_value accel_y;
	struct sensor_value accel_z;
	struct sensor_value gyro_x;
	struct sensor_value gyro_y;
	struct sensor_value gyro_z;
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

static const struct gpio_text_pair_desc g_gpio_text_pairs[] = {
	{ { { 1, 31 }, { 1, 3 } } },
	{ { { 1, 30 }, { 1, 7 } } },
	{ { { 0, 0 }, { 0, 3 } } },
	{ { { 0, 1 }, { 0, 4 } } },
	{ { { 0, 2 }, { 0, 5 } } },
	{ { { 1, 6 }, { 3, 10 } } },
	{ { { 1, 5 }, { 3, 9 } } },
	{ { { 1, 4 }, { 3, 11 } } },
	{ { { 3, 0 }, { 3, 7 } } },
	{ { { 3, 1 }, { 3, 6 } } },
	{ { { 3, 2 }, { 3, 5 } } },
	{ { { 3, 3 }, { 1, 1 } } },
	{ { { 3, 4 }, { 1, 2 } } },
};

static struct gpio_text_pair_state g_gpio_text_pair_state[ARRAY_SIZE(g_gpio_text_pairs)];

static const struct gpio_dt_spec g_sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback g_sw0_cb_data;
static bool g_keywake_ready;
static uint32_t g_keywake_irq_count;
static bool g_sleepi_system_off_pending;

#if DT_NODE_EXISTS(DT_ALIAS(imu0))
static const struct device *const g_imu_dev = DEVICE_DT_GET(DT_ALIAS(imu0));
static struct imu_sample_cache g_imu_latest_sample;
static K_MUTEX_DEFINE(g_imu_sample_lock);
static atomic_t g_imu_sample_ready;
static atomic_t g_imu_stream_started;
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

static const struct device *const g_gpio0_dev =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio0));
static const struct device *const g_gpio1_dev =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio1));
static const struct device *const g_gpio2_dev =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio2));
static const struct device *const g_gpio3_dev =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio3));
static const struct device *const g_uart20_dev =
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(uart20));

static bool g_uart20_test_enabled;
static char g_uart20_rx_buf[FACTORY_UART20_BUF_SIZE];
static size_t g_uart20_rx_len;
static bool g_parser_debug_logging_enabled;
static bool g_multi_uart_output_enabled;

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
static bool g_ble_text_scan_active;
static struct ble_scan_stats g_ble_scan_stats;
static const struct bt_le_scan_param g_ble_scan_param = {
	.type = BT_LE_SCAN_TYPE_ACTIVE,
	.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
};
#endif

static void uart_emit_char(uint8_t ch)
{
	if (g_ctx.uart != NULL && device_is_ready(g_ctx.uart)) {
		uart_poll_out(g_ctx.uart, ch);
	}

	if (g_multi_uart_output_enabled &&
	    g_ctx.uart_primary != NULL && g_ctx.uart_primary != g_ctx.uart &&
	    device_is_ready(g_ctx.uart_primary)) {
		uart_poll_out(g_ctx.uart_primary, ch);
	}

	if (g_multi_uart_output_enabled &&
	    g_uart20_dev != NULL && g_uart20_dev != g_ctx.uart &&
	    g_uart20_dev != g_ctx.uart_primary && device_is_ready(g_uart20_dev)) {
		uart_poll_out(g_uart20_dev, ch);
	}
}

static void uart_send_str(const char *s)
{
	while (*s != '\0') {
		uart_emit_char((uint8_t)*s++);
	}
}

static void uart_send_line(const char *s)
{
	uart_send_str(s);
	uart_send_str("\r\n");
}

static void uart_send_str_dev(const struct device *uart, const char *s)
{
	if (uart == NULL) {
		return;
	}

	while (*s != '\0') {
		uart_poll_out(uart, (uint8_t)*s++);
	}
}

static void uart_send_line_dev(const struct device *uart, const char *s)
{
	uart_send_str_dev(uart, s);
	uart_send_str_dev(uart, "\r\n");
}

static void uart_send_u32(uint32_t value)
{
	char buf[11];
	int i = 0;

	if (value == 0u) {
		uart_emit_char('0');
		return;
	}

	while (value > 0u && i < (int)sizeof(buf)) {
		buf[i++] = (char)('0' + (value % 10u));
		value /= 10u;
	}

	while (i > 0) {
		uart_emit_char((uint8_t)buf[--i]);
	}
}

static void uart_send_s32(int32_t value)
{
	if (value < 0) {
		uart_emit_char('-');
		uart_send_u32((uint32_t)(-(int64_t)value));
		return;
	}

	uart_send_u32((uint32_t)value);
}

static void debug_send_label(const char *label)
{
	if (!g_parser_debug_logging_enabled) {
		return;
	}
	uart_send_str("[DBG] ");
	uart_send_str(label);
}

static void debug_send_line(const char *label, const char *value)
{
	if (!g_parser_debug_logging_enabled) {
		return;
	}
	debug_send_label(label);
	uart_send_line(value);
}

static void debug_send_span(const char *label, const char *value, size_t len)
{
	if (!g_parser_debug_logging_enabled) {
		return;
	}
	debug_send_label(label);
	for (size_t i = 0; i < len; ++i) {
		uart_emit_char((uint8_t)value[i]);
	}
	uart_send_str("\r\n");
}

static void debug_send_u32(const char *label, uint32_t value)
{
	if (!g_parser_debug_logging_enabled) {
		return;
	}
	debug_send_label(label);
	uart_send_u32(value);
	uart_send_str("\r\n");
}

static void debug_send_s32(const char *label, int32_t value)
{
	if (!g_parser_debug_logging_enabled) {
		return;
	}
	debug_send_label(label);
	uart_send_s32(value);
	uart_send_str("\r\n");
}

static void imu_trace_line(const char *stage)
{
#if CONFIG_FACTORY_IMU_TRACE
	uart_send_str("[IMU] ");
	uart_send_line(stage);
#else
	ARG_UNUSED(stage);
#endif
}

static void imu_trace_rc(const char *stage, int rc)
{
#if CONFIG_FACTORY_IMU_TRACE
	uart_send_str("[IMU] ");
	uart_send_str(stage);
	uart_send_str(" rc=");
	uart_send_s32(rc);
	uart_send_str("\r\n");
#else
	ARG_UNUSED(stage);
	ARG_UNUSED(rc);
#endif
}

#if DT_NODE_EXISTS(DT_ALIAS(imu0))
static int at_imu_store_latest_sample(const struct device *dev)
{
	struct imu_sample_cache sample;
	int ret;

	ret = sensor_sample_fetch(dev);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &sample.accel_x);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &sample.accel_y);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &sample.accel_z);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &sample.gyro_x);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &sample.gyro_y);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &sample.gyro_z);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(&g_imu_sample_lock, K_FOREVER);
	g_imu_latest_sample = sample;
	k_mutex_unlock(&g_imu_sample_lock);
	atomic_set(&g_imu_sample_ready, 1);

	return 0;
}

static int at_imu_set_sampling_freq(const struct device *dev)
{
	struct sensor_value odr_attr = {
		.val1 = 26,
		.val2 = 0,
	};
	int ret;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	return ret;
}

#ifdef CONFIG_LSM6DSL_TRIGGER
static void at_imu_trigger_handler(const struct device *dev,
				   const struct sensor_trigger *trig)
{
	int ret;

	ARG_UNUSED(trig);

	ret = at_imu_store_latest_sample(dev);
	if (ret != 0) {
		atomic_set(&g_imu_sample_ready, 0);
	}
}
#endif

static int at_start_imu_stream_if_needed(void)
{
#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	int ret;

	if (!atomic_cas(&g_imu_stream_started, 0, 1)) {
		imu_trace_line("stream:already_started");
		return 0;
	}

	atomic_set(&g_imu_sample_ready, 0);

	ret = at_imu_set_sampling_freq(g_imu_dev);
	imu_trace_rc("stream:set_sampling_freq", ret);
	if (ret != 0) {
		atomic_set(&g_imu_stream_started, 0);
		return ret;
	}

	ret = sensor_trigger_set(g_imu_dev, &trig, at_imu_trigger_handler);
	imu_trace_rc("stream:trigger_set", ret);
	if (ret != 0) {
		atomic_set(&g_imu_stream_started, 0);
		return ret;
	}

	return 0;
#else
	return at_imu_store_latest_sample(g_imu_dev);
#endif
}
#endif

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
	case -EBUSY:
		return "HW_BUSY";
	case -ETIMEDOUT:
		return "HW_TIMEOUT";
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
	debug_send_s32("PARSE:final_rc=", rc);
	debug_send_line("PARSE:final_reason=", error_reason_from_rc(rc));

	if (rc == 0) {
		uart_send_line("OK");
		return;
	}

	uart_send_str("ERROR:");
	uart_send_line(error_reason_from_rc(rc));
}

static bool parse_u32_token(const char *token, uint32_t *value)
{
	uint32_t acc = 0U;

	if (token == NULL || *token == '\0' || value == NULL) {
		return false;
	}

	for (const char *p = token; *p != '\0'; ++p) {
		if (!isdigit((unsigned char)*p)) {
			return false;
		}

		acc = (acc * 10U) + (uint32_t)(*p - '0');
	}

	*value = acc;
	return true;
}

static bool parse_gpio_group_token(const char *token, uint32_t *group)
{
	if (token == NULL || group == NULL) {
		return false;
	}

	if (strncmp(token, "gpio", strlen("gpio")) != 0) {
		return false;
	}

	return parse_u32_token(token + strlen("gpio"), group);
}

static const struct device *gpio_device_from_group(uint32_t group)
{
	switch (group) {
#if DT_NODE_EXISTS(DT_NODELABEL(gpio0))
	case 0U:
		return g_gpio0_dev;
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(gpio1))
	case 1U:
		return g_gpio1_dev;
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(gpio2))
	case 2U:
		return g_gpio2_dev;
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(gpio3))
	case 3U:
		return g_gpio3_dev;
#endif
	default:
		return NULL;
	}
}

static bool gpio_text_endpoint_matches(const struct gpio_text_endpoint *endpoint,
				       uint32_t group, uint32_t pin)
{
	return endpoint != NULL && endpoint->group == group &&
	       endpoint->pin == (gpio_pin_t)pin;
}

static int gpio_text_find_pair(uint32_t group, uint32_t pin, size_t *pair_idx,
			       uint8_t *endpoint_idx)
{
	for (size_t i = 0; i < ARRAY_SIZE(g_gpio_text_pairs); ++i) {
		for (uint8_t j = 0; j < ARRAY_SIZE(g_gpio_text_pairs[i].endpoints); ++j) {
			if (gpio_text_endpoint_matches(&g_gpio_text_pairs[i].endpoints[j],
						       group, pin)) {
				if (pair_idx != NULL) {
					*pair_idx = i;
				}
				if (endpoint_idx != NULL) {
					*endpoint_idx = j;
				}
				return 0;
			}
		}
	}

	return -ENOENT;
}

static int gpio_text_configure_endpoint(const struct gpio_text_endpoint *endpoint,
					gpio_flags_t flags)
{
	const struct device *gpio_dev;

	if (endpoint == NULL) {
		return -EINVAL;
	}

	gpio_dev = gpio_device_from_group(endpoint->group);
	if (gpio_dev == NULL || !device_is_ready(gpio_dev)) {
		return -ENODEV;
	}

	return gpio_pin_configure(gpio_dev, endpoint->pin, flags);
}

static int gpio_text_read_endpoint(const struct gpio_text_endpoint *endpoint, int *value)
{
	const struct device *gpio_dev;
	int rc;

	if (endpoint == NULL || value == NULL) {
		return -EINVAL;
	}

	gpio_dev = gpio_device_from_group(endpoint->group);
	if (gpio_dev == NULL || !device_is_ready(gpio_dev)) {
		return -ENODEV;
	}

	rc = gpio_pin_get(gpio_dev, endpoint->pin);
	if (rc < 0) {
		return -ENODEV;
	}

	*value = rc;
	return 0;
}

static int tokenize_text_command(char *buf, char *argv[], size_t argv_len)
{
	size_t argc = 0U;
	char *cursor = buf;

	if (buf == NULL || argv == NULL || argv_len == 0U) {
		return -EINVAL;
	}

	while (*cursor != '\0') {
		while (isspace((unsigned char)*cursor)) {
			*cursor++ = '\0';
		}

		if (*cursor == '\0') {
			break;
		}

		if (argc >= argv_len) {
			return -EINVAL;
		}

		argv[argc++] = cursor;
		while (*cursor != '\0' && !isspace((unsigned char)*cursor)) {
			cursor++;
		}
	}

	return (int)argc;
}

static void uart_send_fixed6_from_micro(int64_t micro_value)
{
	uint64_t magnitude;
	uint32_t frac_digits[6];

	if (micro_value < 0) {
		uart_emit_char('-');
		magnitude = (uint64_t)(-micro_value);
	} else {
		magnitude = (uint64_t)micro_value;
	}

	uart_send_u32((uint32_t)(magnitude / 1000000ULL));
	uart_emit_char('.');
	magnitude %= 1000000ULL;

	for (int i = 5; i >= 0; --i) {
		frac_digits[i] = (uint32_t)(magnitude % 10ULL);
		magnitude /= 10ULL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(frac_digits); ++i) {
		uart_emit_char((uint8_t)('0' + frac_digits[i]));
	}
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

static void uart20_rx_reset(void)
{
	g_uart20_rx_buf[0] = '\0';
	g_uart20_rx_len = 0U;
}

static int at_enable_uart20_test(void)
{
	static const struct uart_config uart20_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	int rc;

	if (g_uart20_dev == NULL || !device_is_ready(g_uart20_dev)) {
		emit_testdata("STATE1", "UART20TEST", "0", "bool",
			      "uart20_not_ready", "err:HW_NOT_READY");
		return -ENODEV;
	}

	rc = uart_configure(g_uart20_dev, &uart20_cfg);
	if (rc != 0) {
		emit_testdata("STATE1", "UART20TEST", "0", "bool",
			      "uart20_config_failed", "err:HW_NOT_READY");
		return -ENODEV;
	}

	g_uart20_test_enabled = true;
	uart20_rx_reset();

	emit_testdata("STATE1", "UART20TEST", "1", "bool",
		      "uart20_test_enabled",
		      "uart:uart20;trigger:whoami;reply:NRF54LM20A");
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

	imu_trace_line("power:begin");

#if DT_NODE_EXISTS(DT_NODELABEL(power_en))
	if (!device_is_ready(g_power_en_dev)) {
		imu_trace_line("power:power_en_not_ready");
		return -ENODEV;
	}
	ret = regulator_enable(g_power_en_dev);
	imu_trace_rc("power:power_en_enable", ret);
	if (ret < 0 && ret != -EALREADY) {
		return -ENODEV;
	}
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(imu_vdd))
	if (!device_is_ready(g_imu_vdd_dev)) {
		imu_trace_line("power:imu_vdd_not_ready");
		return -ENODEV;
	}
	ret = regulator_enable(g_imu_vdd_dev);
	imu_trace_rc("power:imu_vdd_enable", ret);
	if (ret < 0 && ret != -EALREADY) {
		return -ENODEV;
	}
#endif

	imu_trace_line("power:settle_20ms");
	k_sleep(K_MSEC(20));
	imu_trace_line("power:done");
	return 0;
}

static int at_ensure_imu_ready(void)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	int ret;

	imu_trace_line("ensure:begin");

	if (device_is_ready(g_imu_dev)) {
		imu_trace_line("ensure:already_ready_skip_power");
		return 0;
	}

	ret = at_enable_imu_power();
	if (ret != 0) {
		imu_trace_rc("ensure:power_failed", ret);
		return ret;
	}

	if (!device_is_ready(g_imu_dev)) {
		imu_trace_line("ensure:device_init_begin");
		ret = device_init(g_imu_dev);
		imu_trace_rc("ensure:device_init", ret);
		if (ret < 0 && ret != -EALREADY) {
			return ret;
		}
	}

	if (!device_is_ready(g_imu_dev)) {
		imu_trace_line("ensure:not_ready_after_init");
		return -ENODEV;
	}

	imu_trace_line("ensure:ready");
	return 0;
#else
	imu_trace_line("ensure:alias_missing");
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
	int64_t deadline = k_uptime_get() + FACTORY_IMU_FIRST_SAMPLE_WAIT_MS;

	imu_trace_line("fetch:begin");

	imu_trace_line("fetch:wait_cached_sample");
	while (!atomic_get(&g_imu_sample_ready) && k_uptime_get() < deadline) {
		k_sleep(K_MSEC(10));
	}

	if (!atomic_get(&g_imu_sample_ready)) {
		imu_trace_line("fetch:cached_sample_timeout");
		return -ETIMEDOUT;
	}

	k_mutex_lock(&g_imu_sample_lock, K_FOREVER);
	*accel_x = g_imu_latest_sample.accel_x;
	*accel_y = g_imu_latest_sample.accel_y;
	*accel_z = g_imu_latest_sample.accel_z;
	*gyro_x = g_imu_latest_sample.gyro_x;
	*gyro_y = g_imu_latest_sample.gyro_y;
	*gyro_z = g_imu_latest_sample.gyro_z;
	k_mutex_unlock(&g_imu_sample_lock);

	imu_trace_line("fetch:done");
	return 0;
#else
	ARG_UNUSED(accel_x);
	ARG_UNUSED(accel_y);
	ARG_UNUSED(accel_z);
	ARG_UNUSED(gyro_x);
	ARG_UNUSED(gyro_y);
	ARG_UNUSED(gyro_z);
	imu_trace_line("fetch:alias_missing");
	return -ENODEV;
#endif
}

static int at_get_imu_sample_direct(struct sensor_value *accel_x,
				    struct sensor_value *accel_y,
				    struct sensor_value *accel_z,
				    struct sensor_value *gyro_x,
				    struct sensor_value *gyro_y,
				    struct sensor_value *gyro_z)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	int ret;

	imu_trace_line("direct:begin");

	ret = at_ensure_imu_ready();
	if (ret != 0) {
		imu_trace_rc("direct:ensure_failed", ret);
		return ret;
	}

	ret = at_start_imu_stream_if_needed();
	if (ret != 0) {
		imu_trace_rc("direct:start_stream_failed", ret);
		return ret;
	}

	ret = at_fetch_imu_sample(accel_x, accel_y, accel_z,
				  gyro_x, gyro_y, gyro_z);
	imu_trace_rc("direct:fetch_done", ret);
	return ret;
#else
	ARG_UNUSED(accel_x);
	ARG_UNUSED(accel_y);
	ARG_UNUSED(accel_z);
	ARG_UNUSED(gyro_x);
	ARG_UNUSED(gyro_y);
	ARG_UNUSED(gyro_z);
	imu_trace_line("direct:alias_missing");
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

static int at_measure_batv(int32_t *bat_mv, int32_t *raw_value, bool *from_pmic)
{
	struct sensor_value bat_voltage;
	int16_t bat_raw;
	int rc;

	if (bat_mv == NULL || raw_value == NULL || from_pmic == NULL) {
		return -EINVAL;
	}

	rc = at_pmic_charger_fetch(NULL, &bat_voltage, NULL);
	if (rc == 0) {
		int64_t bat_uv = sensor_value_to_micro(&bat_voltage);

		*bat_mv = (int32_t)(bat_uv / 1000);
		*raw_value = (int32_t)bat_uv;
		*from_pmic = true;
		return 0;
	}

	rc = at_adc_sample_mv((uint8_t)CONFIG_FACTORY_ADC_BATV_CHANNEL, bat_mv, &bat_raw);
	if (rc != 0) {
		return rc;
	}

	*raw_value = (int32_t)bat_raw;
	*from_pmic = false;
	return 0;
}

static int at_handle_batv(void)
{
	int32_t bat_mv;
	int32_t raw_value;
	bool from_pmic;
	int rc;

	rc = at_measure_batv(&bat_mv, &raw_value, &from_pmic);
	if (rc != 0) {
		emit_testdata("STATE2", "BATV", "0", "mV", "batv_read_failed",
			      "err:HW_NOT_READY");
		return rc;
	}

	if (from_pmic) {
		uart_send_str("+TESTDATA:STATE2,ITEM=BATV,VALUE=");
		uart_send_s32(bat_mv);
		uart_send_str(",UNIT=mV,RAW=");
		uart_send_s32(raw_value);
		uart_send_str(",META=src:pmic_charger;ref_delta_mv:");
		uart_send_u32(CONFIG_FACTORY_BATV_DELTA_MAX_MV);
		uart_send_line(";sensor:gauge_voltage");
		return 0;
	}

	emit_testdata_adc("STATE2", "BATV", bat_mv, (int16_t)raw_value,
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

static void at_ble_scan_reset_stats(void)
{
#if defined(CONFIG_BT)
	memset(&g_ble_scan_stats, 0, sizeof(g_ble_scan_stats));
#endif
}

static void at_ble_scan_export_stats(char *first_addr, size_t first_addr_len,
				     int32_t *first_rssi, char *best_addr,
				     size_t best_addr_len, uint32_t *adv_count,
				     int32_t *best_rssi)
{
#if defined(CONFIG_BT)
	if (first_addr != NULL && first_addr_len > 0U) {
		strncpy(first_addr, "none", first_addr_len);
		first_addr[first_addr_len - 1U] = '\0';
	}

	if (best_addr != NULL && best_addr_len > 0U) {
		strncpy(best_addr, "none", best_addr_len);
		best_addr[best_addr_len - 1U] = '\0';
	}

	if (first_rssi != NULL) {
		*first_rssi = -127;
	}

	if (best_rssi != NULL) {
		*best_rssi = -127;
	}

	if (adv_count != NULL) {
		*adv_count = g_ble_scan_stats.adv_count;
	}

	if (g_ble_scan_stats.have_first) {
		if (first_addr != NULL && first_addr_len > 0U) {
			bt_addr_le_to_str(&g_ble_scan_stats.first_addr, first_addr,
					  first_addr_len);
		}
		if (first_rssi != NULL) {
			*first_rssi = g_ble_scan_stats.first_rssi;
		}
	}

	if (g_ble_scan_stats.have_best) {
		if (best_addr != NULL && best_addr_len > 0U) {
			bt_addr_le_to_str(&g_ble_scan_stats.best_addr, best_addr,
					  best_addr_len);
		}
		if (best_rssi != NULL) {
			*best_rssi = g_ble_scan_stats.best_rssi;
		}
	}
#else
	ARG_UNUSED(first_addr);
	ARG_UNUSED(first_addr_len);
	ARG_UNUSED(first_rssi);
	ARG_UNUSED(best_addr);
	ARG_UNUSED(best_addr_len);
	ARG_UNUSED(adv_count);
	ARG_UNUSED(best_rssi);
#endif
}

static int at_ble_scan_start_session(bool text_session)
{
#if defined(CONFIG_BT)
	int rc;

	rc = at_ble_ensure_ready();
	if (rc != 0) {
		return rc;
	}

	if (text_session && g_ble_text_scan_active) {
		return 0;
	}

	if (!text_session && g_ble_text_scan_active) {
		return -EBUSY;
	}

	at_ble_scan_reset_stats();
	rc = bt_le_scan_start(&g_ble_scan_param, at_ble_scan_cb);
	if (rc == -EALREADY) {
		return text_session && g_ble_text_scan_active ? 0 : -EBUSY;
	}

	if (rc != 0) {
		return -ENODEV;
	}

	if (text_session) {
		g_ble_text_scan_active = true;
	}

	return 0;
#else
	ARG_UNUSED(text_session);
	return -ENODEV;
#endif
}

static int at_ble_scan_stop_session(char *first_addr, size_t first_addr_len,
				    int32_t *first_rssi, char *best_addr,
				    size_t best_addr_len, uint32_t *adv_count,
				    int32_t *best_rssi)
{
#if defined(CONFIG_BT)
	int rc = 0;

	if (!g_ble_text_scan_active) {
		if (first_addr != NULL && first_addr_len > 0U) {
			strncpy(first_addr, "none", first_addr_len);
			first_addr[first_addr_len - 1U] = '\0';
		}
		if (best_addr != NULL && best_addr_len > 0U) {
			strncpy(best_addr, "none", best_addr_len);
			best_addr[best_addr_len - 1U] = '\0';
		}
		if (first_rssi != NULL) {
			*first_rssi = -127;
		}
		if (best_rssi != NULL) {
			*best_rssi = -127;
		}
		if (adv_count != NULL) {
			*adv_count = 0U;
		}
		return 0;
	}

	rc = bt_le_scan_stop();
	if (rc != 0 && rc != -EALREADY) {
		return -ENODEV;
	}

	g_ble_text_scan_active = false;
	at_ble_scan_export_stats(first_addr, first_addr_len, first_rssi,
				 best_addr, best_addr_len, adv_count,
				 best_rssi);
	at_ble_scan_reset_stats();
	return 0;
#else
	ARG_UNUSED(first_addr);
	ARG_UNUSED(first_addr_len);
	ARG_UNUSED(first_rssi);
	ARG_UNUSED(best_addr);
	ARG_UNUSED(best_addr_len);
	ARG_UNUSED(adv_count);
	ARG_UNUSED(best_rssi);
	return -ENODEV;
#endif
}

static void emit_text_ble_scan_results(const char *first_addr, int32_t first_rssi,
				       const char *best_addr, int32_t best_rssi,
				       uint32_t adv_count)
{
	if (adv_count == 0U) {
		return;
	}

	uart_send_str("[DEVICE] ");
	uart_send_str(first_addr);
	uart_send_str(" RSSI ");
	uart_send_s32(first_rssi);
	uart_send_str("\r\n");

	if (strcmp(best_addr, first_addr) != 0) {
		uart_send_str("[DEVICE] ");
		uart_send_str(best_addr);
		uart_send_str(" RSSI ");
		uart_send_s32(best_rssi);
		uart_send_str("\r\n");
	}
}

static int at_ble_scan_collect(char *first_addr, size_t first_addr_len,
			       int32_t *first_rssi, char *best_addr,
			       size_t best_addr_len, uint32_t *adv_count,
			       int32_t *best_rssi)
{
#if defined(CONFIG_BT)
	int rc;

	if (first_addr == NULL || best_addr == NULL || first_rssi == NULL ||
	    adv_count == NULL || best_rssi == NULL) {
		return -EINVAL;
	}

	rc = at_ble_scan_start_session(false);
	if (rc != 0) {
		return rc;
	}

	k_msleep(CONFIG_FACTORY_BLE_SCAN_WINDOW_MS);
	rc = bt_le_scan_stop();
	if (rc != 0 && rc != -EALREADY) {
		return -ENODEV;
	}

	at_ble_scan_export_stats(first_addr, first_addr_len, first_rssi,
				 best_addr, best_addr_len, adv_count,
				 best_rssi);
	return 0;
#else
	ARG_UNUSED(first_addr);
	ARG_UNUSED(first_addr_len);
	ARG_UNUSED(first_rssi);
	ARG_UNUSED(best_addr);
	ARG_UNUSED(best_addr_len);
	ARG_UNUSED(adv_count);
	ARG_UNUSED(best_rssi);
	return -ENODEV;
#endif
}

static int at_handle_blescan(void)
{
#if defined(CONFIG_BT)
	char first_addr[BT_ADDR_LE_STR_LEN] = "none";
	char best_addr[BT_ADDR_LE_STR_LEN] = "none";
	int32_t first_rssi;
	int32_t best_rssi = -127;
	uint32_t adv_count = 0U;
	int rc;

	rc = at_ble_scan_collect(first_addr, sizeof(first_addr), &first_rssi,
				 best_addr, sizeof(best_addr), &adv_count,
				 &best_rssi);
	if (rc != 0) {
		emit_testdata("STATE3", "BLESCAN", "0", "adv", "scan_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}
	ARG_UNUSED(first_rssi);

	uart_send_str("+TESTDATA:STATE3,ITEM=BLESCAN,VALUE=");
	uart_send_u32(adv_count);
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

	imu_trace_line("at_imu6d:begin");
	rc = at_get_imu_sample_direct(&accel_x, &accel_y, &accel_z,
				      &gyro_x, &gyro_y, &gyro_z);
	if (rc != 0) {
		imu_trace_rc("at_imu6d:error", rc);
		emit_testdata("STATE4", "IMU6D", "0", "sample", "imu_not_ready",
			      "err:HW_NOT_READY_OR_TIMEOUT");
		return rc;
	}

	imu_trace_line("at_imu6d:emit_result");
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
	imu_trace_line("at_imu6d:alias_missing");
	emit_testdata("STATE4", "IMU6D", "0", "sample", "imu_alias_missing",
		      "err:HW_NOT_READY");
	return -ENODEV;
#endif
}

static int at_capture_mic_stats(uint32_t sample_seconds,
				struct mic_capture_stats *stats)
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
	uint32_t size = 0U;
	uint32_t current_run = 0U;
	uint32_t total_samples = 0U;
	int64_t sum_abs = 0;
	int32_t peak_abs = 0;
	int32_t min_sample = INT32_MAX;
	int32_t max_sample = INT32_MIN;
	int16_t prev_sample = 0;
	int64_t deadline_ms = 0;
	bool have_prev = false;
	bool captured_any = false;
	int rc;

	if (stats == NULL) {
		return -EINVAL;
	}

	memset(stats, 0, sizeof(*stats));

	rc = at_enable_dmic_power();
	if (rc != 0) {
		return -ENODEV;
	}

	if (!device_is_ready(g_dmic_dev)) {
		return -ENODEV;
	}

	dmic_cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

	rc = dmic_configure(g_dmic_dev, &dmic_cfg);
	if (rc != 0) {
		return -ENODEV;
	}

	rc = dmic_trigger(g_dmic_dev, DMIC_TRIGGER_START);
	if (rc != 0) {
		return -ENODEV;
	}

	rc = dmic_read(g_dmic_dev, 0, &discard_buffer, &size,
		       CONFIG_FACTORY_DMIC_READ_TIMEOUT_MS);
	if (rc == 0 && discard_buffer != NULL) {
		(void)k_mem_slab_free(&g_dmic_mem_slab, discard_buffer);
		discard_buffer = NULL;
	}

	if (sample_seconds > 0U) {
		deadline_ms = k_uptime_get() + ((int64_t)sample_seconds * 1000LL);
	}

	do {
		rc = dmic_read(g_dmic_dev, 0, &buffer, &size,
			       CONFIG_FACTORY_DMIC_READ_TIMEOUT_MS);
		if (rc != 0 || buffer == NULL || size < sizeof(int16_t)) {
			(void)dmic_trigger(g_dmic_dev, DMIC_TRIGGER_STOP);
			return -ENODEV;
		}

		captured_any = true;

		for (uint32_t i = 0; i < (size / sizeof(int16_t)); ++i) {
			int32_t sample = ((int16_t *)buffer)[i];
			int32_t abs_sample = (sample < 0) ? -sample : sample;

			if (sample < min_sample) {
				min_sample = sample;
			}
			if (sample > max_sample) {
				max_sample = sample;
			}
			if (abs_sample > peak_abs) {
				peak_abs = abs_sample;
			}

			if (have_prev && sample == prev_sample) {
				current_run++;
			} else {
				current_run = 1U;
				prev_sample = (int16_t)sample;
				have_prev = true;
			}

			if (current_run > stats->max_consecutive) {
				stats->max_consecutive = current_run;
			}

			sum_abs += abs_sample;
			total_samples++;
		}

		(void)k_mem_slab_free(&g_dmic_mem_slab, buffer);
		buffer = NULL;
	} while (sample_seconds > 0U && k_uptime_get() < deadline_ms);

	(void)dmic_trigger(g_dmic_dev, DMIC_TRIGGER_STOP);

	if (!captured_any || total_samples == 0U) {
		return -ENODEV;
	}

	stats->sample_count = total_samples;
	stats->peak_abs = peak_abs;
	stats->avg_abs = (int32_t)(sum_abs / total_samples);
	stats->min_sample = min_sample;
	stats->max_sample = max_sample;
	return 0;
#else
	ARG_UNUSED(sample_seconds);
	ARG_UNUSED(stats);
	return -ENODEV;
#endif
}

static int at_handle_micamp(void)
{
	struct mic_capture_stats stats = { 0 };
	int rc;

	rc = at_capture_mic_stats(0U, &stats);
	if (rc != 0) {
		emit_testdata("STATE4", "MICAMP", "0", "abs16", "dmic_read_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	uart_send_str("+TESTDATA:STATE4,ITEM=MICAMP,VALUE=");
	uart_send_s32(stats.peak_abs);
	uart_send_str(",UNIT=abs16,RAW=");
	uart_send_s32(stats.avg_abs);
	uart_send_str(",META=samples:");
	uart_send_u32(stats.sample_count);
	uart_send_str(";rate:");
	uart_send_u32(CONFIG_FACTORY_DMIC_SAMPLE_RATE_HZ);
	uart_send_line(";mode:single_block;discard:first_block");
	return 0;
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

static int at_enter_ship_mode_common(const char *state, const char *phase)
{
	int rc;

	if (g_ctx.regulator_parent == NULL ||
	    !device_is_ready(g_ctx.regulator_parent)) {
		emit_testdata(state, "SHIPMODE", "0", "bool",
			      "regulator_not_ready", "err:HW_NOT_READY");
		return -ENODEV;
	}

	rc = regulator_parent_ship_mode(g_ctx.regulator_parent);
	if (rc != 0) {
		emit_testdata(state, "SHIPMODE", "0", "bool", "ship_mode_failed",
			      "err:HW_NOT_READY");
		return -ENODEV;
	}

	emit_testdata(state, "SHIPMODE", "1", "bool", "ship_mode_entered", phase);
	return 0;
}

static int at_handle_shipmode_a(void)
{
	return at_enter_ship_mode_common("STATE8A", "phase:A");
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
	return at_enter_ship_mode_common("STATE9B", "phase:B");
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
	{ "AT+UART20TEST", at_enable_uart20_test },
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
	uart_send_line("+CMDS:text:help,gpio set/get,bt init,bt scan on/off,sleep mode,ship mode");
	uart_send_line("+CMDS:text:mic capture <sec>,imu get/off,flash <0-255>,bat get,uart20 on");
	uart_send_line("+CMDS:AT,AT+HELP,AT+FLASH?,AT+FLASH=<0|1|2>,AT+THRESH?");
	uart_send_line("+CMDS:AT+STATE1..AT+STATE7,AT+STATE8A,AT+STATE8B,AT+STATE9B");
	uart_send_line("+CMDS:AT+VBUS,AT+V3P3,AT+UARTLOOP,AT+UART20TEST,AT+CHGCUR,AT+BATV");
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
			       bool track_state, const char *table_name)
{
	debug_send_line("PARSE:dispatch_table=", table_name);

	for (size_t i = 0; i < count; ++i) {
		if (at_cmd_equals(trimmed, trimmed_len, table[i].cmd)) {
			int rc = table[i].fn();

			debug_send_line("PARSE:table_match=", table[i].cmd);

			if (rc == 0 && track_state && g_ctx.persist != NULL) {
				g_ctx.persist->state_bitmap |= BIT(i);
				if (factory_storage_save(g_ctx.persist) != 0) {
					rc = -EIO;
				}
			}

			return rc;
		}
	}

	debug_send_line("PARSE:table_miss=", table_name);

	return -ENOENT;
}

static int text_handle_gpio_set(const char *gpio_token, const char *pin_token,
				const char *value_token)
{
	uint32_t group;
	uint32_t pin;
	uint32_t value;
	size_t pair_idx;
	uint8_t endpoint_idx;
	uint8_t input_idx;
	int rc;

	if (!parse_gpio_group_token(gpio_token, &group) ||
	    !parse_u32_token(pin_token, &pin) ||
	    !parse_u32_token(value_token, &value) || value > 1U) {
		return -EINVAL;
	}

	rc = gpio_text_find_pair(group, pin, &pair_idx, &endpoint_idx);
	if (rc != 0) {
		return -EINVAL;
	}

	input_idx = endpoint_idx == 0U ? 1U : 0U;

	rc = gpio_text_configure_endpoint(&g_gpio_text_pairs[pair_idx].endpoints[input_idx],
					 GPIO_INPUT);
	if (rc != 0) {
		return -ENODEV;
	}

	rc = gpio_text_configure_endpoint(&g_gpio_text_pairs[pair_idx].endpoints[endpoint_idx],
					 value != 0U ? GPIO_OUTPUT_ACTIVE :
						      GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		return -ENODEV;
	}

	g_gpio_text_pair_state[pair_idx].configured = true;
	g_gpio_text_pair_state[pair_idx].output_idx = endpoint_idx;

	uart_send_str("gpio set ");
	uart_send_str(gpio_token);
	uart_send_str(" ");
	uart_send_str(pin_token);
	uart_send_str("\r\n");
	return 0;
}

static int text_handle_gpio_get(const char *gpio_token, const char *pin_token)
{
	uint32_t group;
	uint32_t pin;
	size_t pair_idx;
	uint8_t endpoint_idx;
	uint8_t input_idx;
	int value;
	int rc;

	if (!parse_gpio_group_token(gpio_token, &group) ||
	    !parse_u32_token(pin_token, &pin)) {
		return -EINVAL;
	}

	rc = gpio_text_find_pair(group, pin, &pair_idx, &endpoint_idx);
	if (rc != 0) {
		return -EINVAL;
	}

	if (!g_gpio_text_pair_state[pair_idx].configured) {
		return -EACCES;
	}

	input_idx = g_gpio_text_pair_state[pair_idx].output_idx == 0U ? 1U : 0U;
	rc = gpio_text_configure_endpoint(&g_gpio_text_pairs[pair_idx].endpoints[input_idx],
					 GPIO_INPUT);
	if (rc != 0) {
		return -ENODEV;
	}

	rc = gpio_text_read_endpoint(&g_gpio_text_pairs[pair_idx].endpoints[input_idx],
				    &value);
	if (rc != 0) {
		return rc;
	}

	uart_send_str("Value:");
	uart_send_u32((uint32_t)value);
	uart_send_str("\r\n");
	return 0;
}

static int text_handle_bt_init(void)
{
	int rc = at_ble_ensure_ready();

	if (rc != 0) {
		return rc;
	}

	uart_send_line("LMP: version 6.0");
	return 0;
}

static int text_handle_bt_scan_on(void)
{
	return at_ble_scan_start_session(true);
}

static int text_handle_bt_scan_off(void)
{
#if defined(CONFIG_BT)
	char first_addr[BT_ADDR_LE_STR_LEN] = "none";
	char best_addr[BT_ADDR_LE_STR_LEN] = "none";
	int32_t first_rssi = -127;
	int32_t best_rssi = -127;
	uint32_t adv_count = 0U;
	int rc;

	rc = at_ble_scan_stop_session(first_addr, sizeof(first_addr), &first_rssi,
				      best_addr, sizeof(best_addr), &adv_count,
				      &best_rssi);
	if (rc != 0) {
		return rc;
	}

	emit_text_ble_scan_results(first_addr, first_rssi, best_addr, best_rssi,
				   adv_count);
	uart_send_line("bt scan off");
	return 0;
#else
	return -ENODEV;
#endif
}

static int text_handle_sleep_mode(void)
{
	int rc = at_handle_sleepi();

	if (rc == 0) {
		uart_send_line("sleep mode");
	}

	return rc;
}

static int text_handle_ship_mode(void)
{
	int rc = at_enter_ship_mode_common("TEXT", "phase:text");

	if (rc == 0) {
		uart_send_line("ship mode");
	}

	return rc;
}

static int text_handle_mic_capture(const char *seconds_token)
{
	struct mic_capture_stats stats = { 0 };
	uint32_t seconds = 0U;
	int rc;

	if (!parse_u32_token(seconds_token, &seconds)) {
		return -EINVAL;
	}

	rc = at_capture_mic_stats(seconds, &stats);
	if (rc != 0) {
		return rc;
	}

	uart_send_str("audio data Max:");
	uart_send_s32(stats.max_sample);
	uart_send_str(" Min:");
	uart_send_s32(stats.min_sample);
	uart_send_str(" Max consecutive:");
	uart_send_u32(stats.max_consecutive);
	uart_send_str("\r\n");
	return 0;
}

static int text_handle_imu_get(void)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	struct sensor_value accel_x, accel_y, accel_z;
	struct sensor_value gyro_x, gyro_y, gyro_z;
	int rc;

	imu_trace_line("text_imu_get:begin");
	rc = at_get_imu_sample_direct(&accel_x, &accel_y, &accel_z,
				      &gyro_x, &gyro_y, &gyro_z);
	if (rc != 0) {
		imu_trace_rc("text_imu_get:error", rc);
		return rc;
	}

	imu_trace_line("text_imu_get:emit_result");
	uart_send_str("accel data:");
	uart_send_fixed6_from_micro(sensor_value_to_micro(&accel_x));
	uart_send_str(",");
	uart_send_fixed6_from_micro(sensor_value_to_micro(&accel_y));
	uart_send_str(",");
	uart_send_fixed6_from_micro(sensor_value_to_micro(&accel_z));
	uart_send_str("\r");

	uart_send_str("gyro data: ");
	uart_send_fixed6_from_micro(sensor_value_to_micro(&gyro_x));
	uart_send_str(",");
	uart_send_fixed6_from_micro(sensor_value_to_micro(&gyro_y));
	uart_send_str(",");
	uart_send_fixed6_from_micro(sensor_value_to_micro(&gyro_z));
	uart_send_str("\r\n");
	return 0;
#else
	imu_trace_line("text_imu_get:alias_missing");
	return -ENODEV;
#endif
}

static int text_handle_imu_off(void)
{
	uart_send_line("imu off");
	return 0;
}

static int text_handle_flash_write(const char *value_token)
{
	struct factory_persist verify;
	uint32_t flash_value;
	uint32_t old_value;
	int rc;

	if (!parse_u32_token(value_token, &flash_value) || flash_value > 255U) {
		return -EINVAL;
	}

	if (g_ctx.persist == NULL) {
		return -EACCES;
	}

	old_value = g_ctx.persist->legacy_flash_value;
	g_ctx.persist->legacy_flash_value = flash_value;

	rc = factory_storage_save(g_ctx.persist);
	if (rc != 0) {
		g_ctx.persist->legacy_flash_value = old_value;
		return -EIO;
	}

	rc = factory_storage_load(&verify);
	if (rc != 0 || verify.legacy_flash_value != flash_value) {
		g_ctx.persist->legacy_flash_value = old_value;
		return -EIO;
	}

	uart_send_str("flash ");
	uart_send_u32(flash_value);
	uart_send_line(" OK");
	return 0;
}

static int text_handle_bat_get(void)
{
	int32_t bat_mv;
	int32_t raw_value;
	bool from_pmic;
	int rc;

	rc = at_measure_batv(&bat_mv, &raw_value, &from_pmic);
	if (rc != 0) {
		return rc;
	}
	ARG_UNUSED(raw_value);
	ARG_UNUSED(from_pmic);

	uart_send_str("bat:");
	uart_send_s32(bat_mv);
	uart_send_line("mv");
	return 0;
}

static int text_handle_uart20_on(void)
{
	int rc = at_enable_uart20_test();

	if (rc == 0) {
		uart_send_line("uart20 on");
	}

	return rc;
}

static int dispatch_text_command(char *cmd_buf)
{
	char *argv[FACTORY_TEXT_CMD_MAX_TOKENS] = { 0 };
	int argc = tokenize_text_command(cmd_buf, argv, ARRAY_SIZE(argv));

	debug_send_s32("PARSE:text_argc=", argc);

	if (argc <= 0) {
		debug_send_line("PARSE:text_result=", "empty_or_invalid");
		return -ENOENT;
	}

	if (g_parser_debug_logging_enabled) {
		for (int i = 0; i < argc; ++i) {
			debug_send_label("PARSE:text_argv[");
			uart_send_u32((uint32_t)i);
			uart_send_str("]=");
			uart_send_line(argv[i]);
		}
	}

	if (argc == 1 && strcmp(argv[0], "help") == 0) {
		debug_send_line("PARSE:text_match=", "help");
		return at_handle_help();
	}

	if (argc == 5 && strcmp(argv[0], "gpio") == 0 &&
	    strcmp(argv[1], "set") == 0) {
		debug_send_line("PARSE:text_match=", "gpio set");
		return text_handle_gpio_set(argv[2], argv[3], argv[4]);
	}

	if (argc == 4 && strcmp(argv[0], "gpio") == 0 &&
	    strcmp(argv[1], "get") == 0) {
		debug_send_line("PARSE:text_match=", "gpio get");
		return text_handle_gpio_get(argv[2], argv[3]);
	}

	if (argc == 2 && strcmp(argv[0], "bt") == 0 &&
	    strcmp(argv[1], "init") == 0) {
		debug_send_line("PARSE:text_match=", "bt init");
		return text_handle_bt_init();
	}

	if (argc == 3 && strcmp(argv[0], "bt") == 0 &&
	    strcmp(argv[1], "scan") == 0 && strcmp(argv[2], "on") == 0) {
		debug_send_line("PARSE:text_match=", "bt scan on");
		return text_handle_bt_scan_on();
	}

	if (argc == 3 && strcmp(argv[0], "bt") == 0 &&
	    strcmp(argv[1], "scan") == 0 && strcmp(argv[2], "off") == 0) {
		debug_send_line("PARSE:text_match=", "bt scan off");
		return text_handle_bt_scan_off();
	}

	if (argc == 2 && strcmp(argv[0], "sleep") == 0 &&
	    strcmp(argv[1], "mode") == 0) {
		debug_send_line("PARSE:text_match=", "sleep mode");
		return text_handle_sleep_mode();
	}

	if (argc == 2 && strcmp(argv[0], "sys") == 0 &&
	    strcmp(argv[1], "off") == 0) {
		debug_send_line("PARSE:text_match=", "sys off");
		return text_handle_sleep_mode();
	}

	if (argc == 2 && strcmp(argv[0], "ship") == 0 &&
	    strcmp(argv[1], "mode") == 0) {
		debug_send_line("PARSE:text_match=", "ship mode");
		return text_handle_ship_mode();
	}

	if (argc == 3 && strcmp(argv[0], "mic") == 0 &&
	    strcmp(argv[1], "capture") == 0) {
		debug_send_line("PARSE:text_match=", "mic capture");
		return text_handle_mic_capture(argv[2]);
	}

	if (argc == 2 && strcmp(argv[0], "imu") == 0 &&
	    strcmp(argv[1], "get") == 0) {
		debug_send_line("PARSE:text_match=", "imu get");
		return text_handle_imu_get();
	}

	if (argc == 2 && strcmp(argv[0], "imu") == 0 &&
	    strcmp(argv[1], "off") == 0) {
		debug_send_line("PARSE:text_match=", "imu off");
		return text_handle_imu_off();
	}

	if (argc == 2 && strcmp(argv[0], "flash") == 0) {
		debug_send_line("PARSE:text_match=", "flash");
		return text_handle_flash_write(argv[1]);
	}

	if (argc == 2 && strcmp(argv[0], "bat") == 0 &&
	    strcmp(argv[1], "get") == 0) {
		debug_send_line("PARSE:text_match=", "bat get");
		return text_handle_bat_get();
	}

	if (argc == 2 && strcmp(argv[0], "uart20") == 0 &&
	    strcmp(argv[1], "on") == 0) {
		debug_send_line("PARSE:text_match=", "uart20 on");
		return text_handle_uart20_on();
	}

	debug_send_line("PARSE:text_result=", "no_match");

	return -ENOENT;
}

void at_handler_init(const struct device *uart_dev,
		     const struct device *regulator_parent,
		     struct factory_persist *persist)
{
	g_ctx.uart = uart_dev;
	g_ctx.uart_primary = uart_dev;
	g_ctx.regulator_parent = regulator_parent;
	g_ctx.persist = persist;
	g_parser_debug_logging_enabled = false;
	g_multi_uart_output_enabled = false;
	g_adc_initialized = false;
	g_keywake_ready = false;
	g_keywake_irq_count = 0;
	g_sleepi_system_off_pending = false;
	g_uart20_test_enabled = false;
	uart20_rx_reset();
	memset(g_gpio_text_pair_state, 0, sizeof(g_gpio_text_pair_state));
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	memset(&g_imu_latest_sample, 0, sizeof(g_imu_latest_sample));
	atomic_set(&g_imu_sample_ready, 0);
	atomic_set(&g_imu_stream_started, 0);
#endif
#if defined(CONFIG_BT)
	g_ble_text_scan_active = false;
	at_ble_scan_reset_stats();
#endif
}

void at_handler_early_init(void)
{
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
	int ret;

	ret = at_enable_imu_power();
	if (ret != 0) {
		return;
	}

	if (!device_is_ready(g_imu_dev)) {
		ret = device_init(g_imu_dev);
		if (ret < 0 && ret != -EALREADY) {
			return;
		}
	}

	if (!device_is_ready(g_imu_dev)) {
		return;
	}

	(void)at_start_imu_stream_if_needed();
#endif
}

bool at_handler_uart20_service_enabled(void)
{
	return g_uart20_test_enabled;
}

void at_handler_poll_background(void)
{
	uint8_t ch;

	if (!g_uart20_test_enabled || g_uart20_dev == NULL ||
	    !device_is_ready(g_uart20_dev)) {
		return;
	}

	while (uart_poll_in(g_uart20_dev, &ch) == 0) {
		if (ch == '\r' || ch == '\n') {
			if (g_uart20_rx_len > 0U) {
				g_uart20_rx_buf[g_uart20_rx_len] = '\0';
				if (strcmp(g_uart20_rx_buf, "whoami") == 0) {
					uart_send_line_dev(g_uart20_dev, "NRF54LM20A");
				}
				uart20_rx_reset();
			}
			continue;
		}

		if (ch < 0x20U || ch > 0x7eU) {
			uart20_rx_reset();
			continue;
		}

		if (g_uart20_rx_len < (FACTORY_UART20_BUF_SIZE - 1U)) {
			g_uart20_rx_buf[g_uart20_rx_len++] = (char)ch;
		} else {
			uart20_rx_reset();
		}
	}
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

void at_handler_process_line_from_uart(const struct device *uart_dev,
				       const char *line, size_t len)
{
	const char *trimmed = line;
	size_t trimmed_len = len;
	char text_cmd_buf[FACTORY_CMD_PARSE_BUF_SIZE];
	int rc;

	if (uart_dev != NULL && device_is_ready(uart_dev)) {
		g_ctx.uart = uart_dev;
	} else {
		g_ctx.uart = g_ctx.uart_primary;
	}

	if (g_ctx.uart == g_ctx.uart_primary) {
		debug_send_line("PARSE:source_uart=", "uart21");
	} else if (g_ctx.uart == g_uart20_dev) {
		debug_send_line("PARSE:source_uart=", "uart20");
	} else {
		debug_send_line("PARSE:source_uart=", "other");
	}

	if (line == NULL || len == 0) {
		debug_send_line("PARSE:skip=", "null_or_empty");
		return;
	}

	debug_send_u32("PARSE:raw_len=", (uint32_t)len);
	debug_send_span("PARSE:raw=", line, len);

	while (trimmed_len > 0 && isspace((unsigned char)trimmed[trimmed_len - 1])) {
		trimmed_len--;
	}

	while (trimmed_len > 0 && isspace((unsigned char)*trimmed)) {
		trimmed++;
		trimmed_len--;
	}

	if (trimmed_len == 0) {
		debug_send_line("PARSE:skip=", "trimmed_empty");
		return;
	}

	debug_send_u32("PARSE:trimmed_len=", (uint32_t)trimmed_len);
	debug_send_span("PARSE:trimmed=", trimmed, trimmed_len);

	/* Bare "AT" command is no longer handled (V3: text command protocol, not AT framework). */

	if (at_cmd_equals(trimmed, trimmed_len, "AT+HELP")) {
		debug_send_line("PARSE:direct_match=", "AT+HELP");
		rc = at_handle_help();
		emit_final_status(rc);
		return;
	}

	if (at_cmd_equals(trimmed, trimmed_len, "AT+FLASH?")) {
		debug_send_line("PARSE:direct_match=", "AT+FLASH?");
		rc = at_handle_flash_get();
		emit_final_status(rc);
		return;
	}

	if (at_cmd_equals(trimmed, trimmed_len, "AT+THRESH?")) {
		debug_send_line("PARSE:direct_match=", "AT+THRESH?");
		rc = at_handle_thresh_get();
		emit_final_status(rc);
		return;
	}

	if (trimmed_len >= strlen("AT+FLASH=") &&
	    strncmp(trimmed, "AT+FLASH=", strlen("AT+FLASH=")) == 0) {
		debug_send_line("PARSE:direct_match=", "AT+FLASH=");
		debug_send_span("PARSE:flash_arg=",
			       trimmed + strlen("AT+FLASH="),
			       trimmed_len - strlen("AT+FLASH="));
		rc = at_handle_flash_set(trimmed + strlen("AT+FLASH="));
		emit_final_status(rc);
		return;
	}

	rc = dispatch_from_table(trimmed, trimmed_len,
				 g_item_cmds, ARRAY_SIZE(g_item_cmds), false,
				 "item_table");
	if (rc != -ENOENT) {
		emit_final_status(rc);
		return;
	}

	rc = dispatch_from_table(trimmed, trimmed_len,
				 g_state_cmds, ARRAY_SIZE(g_state_cmds), true,
				 "state_table");
	if (rc != -ENOENT) {
		emit_final_status(rc);
		return;
	}

	if (trimmed_len < sizeof(text_cmd_buf)) {
		debug_send_line("PARSE:dispatch_table=", "text_table");
		memcpy(text_cmd_buf, trimmed, trimmed_len);
		text_cmd_buf[trimmed_len] = '\0';
		rc = dispatch_text_command(text_cmd_buf);
		if (rc != -ENOENT) {
			/* V3 text commands use their own response format;
			 * only emit error status on failure, no trailing OK. */
			if (rc != 0) {
				emit_final_status(rc);
			}
			return;
		}
		debug_send_line("PARSE:text_table=", "miss");
	} else {
		debug_send_line("PARSE:text_table=", "skipped_line_too_long");
	}

	emit_final_status(-ENOENT);
}
