#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

/* ========== UART Configuration ========== */

#define UART_NODE DT_NODELABEL(uart20)
#define LED_NODE  DT_ALIAS(led0)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* ========== Flash Storage Layout ========== */
/* Use the last page of flash for factory test flags.
 * Fixed partition "factory_data" should be defined in overlay,
 * or we use a known safe offset. For simplicity we use the
 * scratch/storage partition if available, otherwise define a
 * dedicated area.
 */

#define FACTORY_MAGIC 0xFA51B007

/* Persistent data structure stored in flash */
struct factory_data {
	uint32_t magic;		 /* Magic number to validate data */
	uint32_t test_result;	 /* Bitmask: each bit = one test pass(1)/fail(0) */
	uint32_t all_passed;	 /* 1 = all tests verified passed, 0 = not yet */
	uint32_t test_count;	 /* Number of tests registered */
	uint32_t reserved[4];	 /* Reserved for future use */
};

/* Flash page to use - we'll use a fixed area.
 * On nRF54LM20A, we use the last sector of the application flash.
 * The user should define a partition in the overlay for safety.
 */
#define FACTORY_FLASH_OFFSET 0xFE000 /* Near end of 1MB flash, 8KB before end */
#define FACTORY_FLASH_SIZE   FLASH_AREA_LABEL_STORAGE_SIZE

static const struct device *flash_dev =
	DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));

/* ========== AT Command Buffer ========== */

#define AT_BUF_SIZE 128
#define AT_CMD_MAX_ARGS 8

static char at_buf[AT_BUF_SIZE];
static uint16_t at_buf_len;

/* ========== Test Framework ========== */

#define MAX_TESTS 32

typedef bool (*test_func_t)(void);

struct test_entry {
	const char *name;     /* Test name, e.g. "GPIO", "I2C" */
	const char *cmd;      /* AT command to trigger, e.g. "AT+GPIO" */
	test_func_t func;     /* Test function, returns true=pass, false=fail */
	uint8_t bit_index;    /* Bit position in test_result bitmask */
};

static struct test_entry test_registry[MAX_TESTS];
static uint8_t test_count;
static struct factory_data fdata;

/* Register a test - called at init */
static void test_register(const char *name, const char *cmd,
			  test_func_t func)
{
	if (test_count >= MAX_TESTS) {
		return;
	}
	test_registry[test_count].name = name;
	test_registry[test_count].cmd = cmd;
	test_registry[test_count].func = func;
	test_registry[test_count].bit_index = test_count;
	test_count++;
}

/* ========== Flash Operations ========== */

static bool factory_flash_read(struct factory_data *data)
{
	if (!device_is_ready(flash_dev)) {
		return false;
	}

	int rc = flash_read(flash_dev, FACTORY_FLASH_OFFSET, data,
			    sizeof(*data));
	if (rc != 0) {
		return false;
	}

	return true;
}

static bool factory_flash_write(struct factory_data *data)
{
	if (!device_is_ready(flash_dev)) {
		return false;
	}

	data->magic = FACTORY_MAGIC;
	data->test_count = test_count;

	/* Erase the page first */
	int rc = flash_erase(flash_dev, FACTORY_FLASH_OFFSET,
			     FLASH_AREA_LABEL(storage_SIZE));
	if (rc != 0) {
		return false;
	}

	/* Write new data */
	rc = flash_write(flash_dev, FACTORY_FLASH_OFFSET, data,
			 sizeof(*data));
	if (rc != 0) {
		return false;
	}

	return true;
}

static bool factory_load(void)
{
	if (!factory_flash_read(&fdata)) {
		memset(&fdata, 0, sizeof(fdata));
		return false;
	}

	if (fdata.magic != FACTORY_MAGIC) {
		memset(&fdata, 0, sizeof(fdata));
		return false;
	}

	return true;
}

static bool factory_save(void)
{
	return factory_flash_write(&fdata);
}

/* ========== UART Helpers ========== */

static void uart_send_str(const char *str)
{
	while (*str) {
		uart_poll_out(uart_dev, (uint8_t)*str++);
	}
}

static void uart_send_line(const char *str)
{
	uart_send_str(str);
	uart_send_str("\r\n");
}

/* Simple integer to string */
static void uart_send_uint(uint32_t val)
{
	char buf[12];
	int i = 0;

	if (val == 0) {
		uart_poll_out(uart_dev, '0');
		return;
	}

	while (val > 0) {
		buf[i++] = '0' + (val % 10);
		val /= 10;
	}

	/* Reverse */
	while (i > 0) {
		uart_poll_out(uart_dev, buf[--i]);
	}
}

/* ========== AT Command Handlers ========== */

static void at_handle_list(void)
{
	uart_send_line("OK");
	uart_send_str("+TESTS:");
	uart_send_uint(test_count);
	uart_send_str("\r\n");

	for (int i = 0; i < test_count; i++) {
		uart_send_str("  ");
		uart_send_uint(i);
		uart_send_str(": ");
		uart_send_str(test_registry[i].cmd);
		uart_send_str(" - ");
		uart_send_line(test_registry[i].name);

		/* Show current result if we have data */
		if (i < 32) {
			bool passed = (fdata.test_result & BIT(i)) != 0;
			uart_send_str("    Status: ");
			uart_send_line(passed ? "PASS" : "NOT TESTED/FAIL");
		}
	}
	uart_send_line("OK");
}

/* Run a single test by index */
static void at_handle_run_test(uint8_t index)
{
	if (index >= test_count) {
		uart_send_line("ERROR:Invalid test index");
		return;
	}

	struct test_entry *t = &test_registry[index];

	uart_send_str("Running test: ");
	uart_send_str(t->name);
	uart_send_str(" (");
	uart_send_str(t->cmd);
	uart_send_str(")\r\n");

	bool passed = t->func();

	if (passed) {
		fdata.test_result |= BIT(t->bit_index);
		uart_send_str("  Result: ");
		uart_send_line("PASS");
	} else {
		fdata.test_result &= ~BIT(t->bit_index);
		uart_send_str("  Result: ");
		uart_send_line("FAIL");
	}

	/* Save to flash immediately */
	if (factory_save()) {
		uart_send_line("  Saved to flash");
	} else {
		uart_send_line("  ERROR:Flash write failed");
	}

	uart_send_line("OK");
}

/* AT+CHECK - Verify all tests passed and set the final flag */
static void at_handle_check(void)
{
	uart_send_line("Checking test results...");

	if (test_count == 0) {
		uart_send_line("ERROR:No tests registered");
		return;
	}

	uint32_t expected_mask = 0;
	for (int i = 0; i < test_count; i++) {
		expected_mask |= BIT(i);
	}

	uart_send_str("  Tests: ");
	uart_send_uint(test_count);
	uart_send_str(", Expected mask: 0x");
	/* Print hex */
	for (int i = 7; i >= 0; i--) {
		uint8_t nibble = (expected_mask >> (i * 4)) & 0xF;
		uart_poll_out(uart_dev,
			      nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
	}
	uart_send_str("\r\n");

	uart_send_str("  Result mask: 0x");
	for (int i = 7; i >= 0; i--) {
		uint8_t nibble = (fdata.test_result >> (i * 4)) & 0xF;
		uart_poll_out(uart_dev,
			      nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
	}
	uart_send_str("\r\n");

	/* Check each test */
	bool all_pass = true;
	for (int i = 0; i < test_count; i++) {
		bool passed = (fdata.test_result & BIT(i)) != 0;
		uart_send_str("  [");
		uart_send_uint(i);
		uart_send_str("] ");
		uart_send_str(test_registry[i].name);
		uart_send_str(": ");
		uart_send_line(passed ? "PASS" : "FAIL");
		if (!passed) {
			all_pass = false;
		}
	}

	if (all_pass && (fdata.test_result & expected_mask) == expected_mask) {
		fdata.all_passed = 1;
		factory_save();
		uart_send_line("ALL TESTS PASSED - Flag saved to flash");
		uart_send_line("OK");
	} else {
		fdata.all_passed = 0;
		factory_save();
		uart_send_line("FAILED - Some tests did not pass");
		uart_send_line("FAIL");
	}
}

/* AT+STATUS - Show current factory data from flash */
static void at_handle_status(void)
{
	uart_send_line("=== Factory Test Status ===");
	uart_send_str("Magic: 0x");
	for (int i = 7; i >= 0; i--) {
		uint8_t nibble = (fdata.magic >> (i * 4)) & 0xF;
		uart_poll_out(uart_dev,
			      nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
	}
	uart_send_str("\r\n");

	uart_send_str("Tests registered: ");
	uart_send_uint(test_count);
	uart_send_str("\r\n");

	uart_send_str("Test result mask: 0x");
	for (int i = 7; i >= 0; i--) {
		uint8_t nibble = (fdata.test_result >> (i * 4)) & 0xF;
		uart_poll_out(uart_dev,
			      nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
	}
	uart_send_str("\r\n");

	uart_send_str("All passed flag: ");
	uart_send_line(fdata.all_passed ? "1 (PASSED)" : "0 (NOT PASSED)");
	uart_send_line("===========================");
	uart_send_line("OK");
}

/* AT+RESET - Reset all test flags */
static void at_handle_reset(void)
{
	memset(&fdata, 0, sizeof(fdata));
	factory_save();
	uart_send_line("Factory data reset");
	uart_send_line("OK");
}

/* AT+RUNALL - Run all registered tests sequentially */
static void at_handle_runall(void)
{
	uart_send_line("Running all tests...");
	bool all_pass = true;

	for (int i = 0; i < test_count; i++) {
		at_handle_run_test(i);
		if (!(fdata.test_result & BIT(i))) {
			all_pass = false;
		}
	}

	uart_send_str("\r\n=== Summary ===\r\n");
	uart_send_line(all_pass ? "ALL PASSED" : "SOME FAILED");
	uart_send_line("OK");
}

/* ========== AT Command Parser ========== */

static void at_process_command(const char *cmd, uint16_t len)
{
	/* Remove trailing \r\n */
	while (len > 0 && (cmd[len - 1] == '\r' || cmd[len - 1] == '\n')) {
		len--;
	}

	/* Empty line */
	if (len == 0) {
		return;
	}

	/* AT alone - basic AT response */
	if (len == 2 && strncmp(cmd, "AT", 2) == 0) {
		uart_send_line("OK");
		return;
	}

	/* AT+LIST - List all registered tests */
	if (strncmp(cmd, "AT+LIST", 7) == 0) {
		at_handle_list();
		return;
	}

	/* AT+STATUS - Show flash status */
	if (strncmp(cmd, "AT+STATUS", 9) == 0) {
		at_handle_status();
		return;
	}

	/* AT+CHECK - Verify all passed and set flag */
	if (strncmp(cmd, "AT+CHECK", 8) == 0) {
		at_handle_check();
		return;
	}

	/* AT+RESET - Clear all flags */
	if (strncmp(cmd, "AT+RESET", 8) == 0) {
		at_handle_reset();
		return;
	}

	/* AT+RUNALL - Run all tests */
	if (strncmp(cmd, "AT+RUNALL", 9) == 0) {
		at_handle_runall();
		return;
	}

	/* AT+TESTx - Run specific test by index */
	if (strncmp(cmd, "AT+TEST", 7) == 0 && len > 7) {
		int idx = 0;
		for (uint16_t i = 7; i < len; i++) {
			if (cmd[i] >= '0' && cmd[i] <= '9') {
				idx = idx * 10 + (cmd[i] - '0');
			} else {
				break;
			}
		}
		at_handle_run_test((uint8_t)idx);
		return;
	}

	/* Try to match registered test commands */
	for (int i = 0; i < test_count; i++) {
		uint16_t cmd_len = strlen(test_registry[i].cmd);
		if (len == cmd_len &&
		    strncmp(cmd, test_registry[i].cmd, cmd_len) == 0) {
			at_handle_run_test(i);
			return;
		}
	}

	uart_send_line("ERROR:Unknown command");
}

/* ========== Test Stubs (empty implementations) ========== */

static bool test_gpio(void)
{
	/* TODO: Implement GPIO test */
	uart_send_line("  [GPIO stub - not implemented]");
	return false;
}

static bool test_i2c(void)
{
	/* TODO: Implement I2C test */
	uart_send_line("  [I2C stub - not implemented]");
	return false;
}

static bool test_spi(void)
{
	/* TODO: Implement SPI test */
	uart_send_line("  [SPI stub - not implemented]");
	return false;
}

static bool test_adc(void)
{
	/* TODO: Implement ADC test */
	uart_send_line("  [ADC stub - not implemented]");
	return false;
}

static bool test_uart(void)
{
	/* TODO: Implement UART test */
	uart_send_line("  [UART stub - not implemented]");
	return false;
}

static bool test_pwm(void)
{
	/* TODO: Implement PWM test */
	uart_send_line("  [PWM stub - not implemented]");
	return false;
}

static bool test_imu(void)
{
	/* TODO: Implement IMU test */
	uart_send_line("  [IMU stub - not implemented]");
	return false;
}

static bool test_dmic(void)
{
	/* TODO: Implement DMIC test */
	uart_send_line("  [DMIC stub - not implemented]");
	return false;
}

static bool test_ble(void)
{
	/* TODO: Implement BLE test */
	uart_send_line("  [BLE stub - not implemented]");
	return false;
}

static bool test_nfc(void)
{
	/* TODO: Implement NFC test */
	uart_send_line("  [NFC stub - not implemented]");
	return false;
}

static bool test_flash(void)
{
	/* TODO: Implement Flash test */
	uart_send_line("  [Flash stub - not implemented]");
	return false;
}

/* ========== Register All Tests ========== */

static void tests_init(void)
{
	test_register("GPIO",  "AT+GPIO",  test_gpio);
	test_register("I2C",   "AT+I2C",   test_i2c);
	test_register("SPI",   "AT+SPI",   test_spi);
	test_register("ADC",   "AT+ADC",   test_adc);
	test_register("UART",  "AT+UART",  test_uart);
	test_register("PWM",   "AT+PWM",   test_pwm);
	test_register("IMU",   "AT+IMU",   test_imu);
	test_register("DMIC",  "AT+DMIC",  test_dmic);
	test_register("BLE",   "AT+BLE",   test_ble);
	test_register("NFC",   "AT+NFC",   test_nfc);
	test_register("FLASH", "AT+FLASH", test_flash);
}

/* ========== Main ========== */

int main(void)
{
	if (!device_is_ready(uart_dev)) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	/* Initialize flash */
	if (!device_is_ready(flash_dev)) {
		uart_send_line("ERROR:Flash device not ready");
		return 0;
	}

	/* Load factory data from flash */
	bool loaded = factory_load();
	if (!loaded) {
		/* First boot or corrupted - initialize */
		memset(&fdata, 0, sizeof(fdata));
	}

	/* Register all tests */
	tests_init();

	/* Welcome message */
	uart_send_line("");
	uart_send_line("=== nRF54LM20A Factory Test ===");
	uart_send_str("Tests registered: ");
	uart_send_uint(test_count);
	uart_send_str("\r\n");
	uart_send_str("Flash data: ");
	uart_send_line(loaded ? "Loaded" : "Initialized (new)");
	uart_send_str("All passed: ");
	uart_send_line(fdata.all_passed ? "YES" : "NO");
	uart_send_line("Send AT commands via UART (1Mbaud)");
	uart_send_line("  AT          - Response OK");
	uart_send_line("  AT+LIST     - List all tests");
	uart_send_line("  AT+STATUS   - Show flash status");
	uart_send_line("  AT+TESTn    - Run test by index");
	uart_send_line("  AT+<CMD>    - Run test by name");
	uart_send_line("  AT+RUNALL   - Run all tests");
	uart_send_line("  AT+CHECK    - Verify & set pass flag");
	uart_send_line("  AT+RESET    - Clear all flags");
	uart_send_line("================================");

	/* Main loop - receive and process AT commands */
	while (1) {
		uint8_t ch;

		if (uart_poll_in(uart_dev, &ch) == 0) {
			if (ch == '\r' || ch == '\n') {
				/* End of command - process it */
				if (at_buf_len > 0) {
					at_buf[at_buf_len] = '\0';
					at_process_command(at_buf, at_buf_len);
					at_buf_len = 0;
				}
			} else if (at_buf_len < AT_BUF_SIZE - 1) {
				/* Store character */
				at_buf[at_buf_len++] = ch;
			} else {
				/* Buffer overflow - reset */
				at_buf_len = 0;
				uart_send_line("ERROR:Buffer overflow");
			}
		}

		k_msleep(2);
	}

	return 0;
}
