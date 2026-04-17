#include "at_handler.h"

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "factory_storage.h"

struct at_ctx {
	const struct device *uart;
	const struct device *regulator_parent;
	struct factory_persist *persist;
};

static struct at_ctx g_ctx;

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

static int at_handle_vbus(void)
{
	return at_handle_not_implemented("STATE1", "VBUS", "err:ADC_NOT_READY");
}

static int at_handle_v3p3(void)
{
	return at_handle_not_implemented("STATE1", "3V3", "err:ADC_NOT_READY");
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
	return at_handle_not_implemented("STATE3", "GPIOLOOP", "err:GPIO_LOOPBACK_TBD");
}

static int at_handle_nfcloop(void)
{
	return at_handle_not_implemented("STATE3", "NFCLOOP", "err:NFC_NOT_READY");
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
	return at_handle_not_implemented("STATE8B", "VBUS_B", "err:ADC_NOT_READY");
}

static int at_handle_v3p3_b(void)
{
	return at_handle_not_implemented("STATE8B", "3V3_B", "err:ADC_NOT_READY");
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
	uart_send_line("+CMDS:AT,AT+HELP,AT+FLASH?,AT+FLASH=<0|1|2>");
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
