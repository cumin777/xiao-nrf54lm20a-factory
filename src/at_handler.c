#include "at_handler.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
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

static int at_handle_not_implemented(const char *state, const char *item,
			     const char *meta)
{
	emit_testdata(state, item, "0", "na", "not_implemented", meta);
	return -ENOSYS;
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

static int at_handle_chgcur(void)
{
	return at_handle_not_implemented("STATE2", "CHGCUR", "err:SENSOR_NOT_READY");
}

static int at_handle_batv(void)
{
	return at_handle_not_implemented("STATE2", "BATV", "err:ADC_NOT_READY");
}

static int at_handle_blescan(void)
{
	return at_handle_not_implemented("STATE3", "BLESCAN", "err:BLE_NOT_READY");
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
	return at_handle_not_implemented("STATE4", "IMU6D", "err:IMU_NOT_READY");
}

static int at_handle_micamp(void)
{
	return at_handle_not_implemented("STATE4", "MICAMP", "err:DMIC_NOT_READY");
}

static int at_handle_sleepi(void)
{
	return at_handle_not_implemented("STATE5", "SLEEPI", "err:MEASURE_PATH_TBD");
}

static int at_handle_keywake(void)
{
	return at_handle_not_implemented("STATE6", "KEYWAKE", "err:KEY_PATH_TBD");
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
