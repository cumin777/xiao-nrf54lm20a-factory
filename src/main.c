#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "at_handler.h"
#include "factory_storage.h"

#define UART_NODE DT_NODELABEL(uart21)
#define UART20_NODE DT_NODELABEL(uart20)
#define LED_NODE DT_ALIAS(led0)
#define REGULATOR_PARENT_NODE DT_ALIAS(regulator_parent)

#if DT_NODE_EXISTS(DT_ALIAS(led1))
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif

#define CMD_BUF_SIZE 160
#define CMD_IDLE_FLUSH_MS 80

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct device *const uart20_dev = DEVICE_DT_GET_OR_NULL(UART20_NODE);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct device *const regulator_parent =
	DEVICE_DT_GET_OR_NULL(REGULATOR_PARENT_NODE);

static struct factory_persist g_persist;
static bool g_led_ready;
static bool g_uart_debug_logging_enabled;
static bool g_dual_uart_diag_enabled;

static bool uart_rx_is_printable(uint8_t ch);
static void uart_rx_reset(char *buf, size_t *len);

static void uart_send_str_dev(const struct device *uart, const char *str)
{
	if (uart == NULL || !device_is_ready(uart)) {
		return;
	}

	while (*str) {
		uart_poll_out(uart, (uint8_t)*str++);
	}
}

static void uart_send_str_all(const char *str)
{
	uart_send_str_dev(uart_dev, str);
	if (g_dual_uart_diag_enabled &&
	    uart20_dev != NULL && uart20_dev != uart_dev) {
		uart_send_str_dev(uart20_dev, str);
	}
}

static void uart_send_line(const char *str)
{
	uart_send_str_all(str);
	uart_send_str_all("\r\n");
}

static void uart_send_u32_all(uint32_t value)
{
	char buf[11];
	int i = 0;

	if (value == 0U) {
		uart_send_str_all("0");
		return;
	}

	while (value > 0U && i < (int)sizeof(buf)) {
		buf[i++] = (char)('0' + (value % 10U));
		value /= 10U;
	}

	while (i > 0) {
		char ch[2] = { buf[--i], '\0' };
		uart_send_str_all(ch);
	}
}

static void uart_send_hex8(uint8_t value)
{
	static const char hex[] = "0123456789ABCDEF";

	char out[3] = {
		hex[(value >> 4) & 0x0F],
		hex[value & 0x0F],
		'\0',
	};

	uart_send_str_all(out);
}

static void debug_send_label(const char *label)
{
	if (!g_uart_debug_logging_enabled) {
		return;
	}
	uart_send_str_all("[DBG] ");
	uart_send_str_all(label);
}

static void debug_send_line(const char *label, const char *value)
{
	if (!g_uart_debug_logging_enabled) {
		return;
	}
	debug_send_label(label);
	uart_send_line(value);
}

static void debug_send_u32(const char *label, uint32_t value)
{
	if (!g_uart_debug_logging_enabled) {
		return;
	}
	debug_send_label(label);
	uart_send_u32_all(value);
	uart_send_str_all("\r\n");
}

static void debug_send_rx_line_tagged(const char *tag, const char *buf, size_t len)
{
	if (!g_uart_debug_logging_enabled) {
		return;
	}
	debug_send_label(tag);

	for (size_t i = 0; i < len; ++i) {
		char ch[2] = { buf[i], '\0' };
		uart_send_str_all(ch);
	}

	uart_send_str_all("\r\n");
}

static void process_uart_rx(const struct device *src_uart, const char *tag,
			    char *buf, size_t *len, int64_t *last_rx_ms,
			    bool *handled)
{
	uint8_t ch;
	int rc;

	if (src_uart == NULL || !device_is_ready(src_uart)) {
		return;
	}

	while ((rc = uart_poll_in(src_uart, &ch)) == 0) {
		*handled = true;
		*last_rx_ms = k_uptime_get();

		if (ch == '\r' || ch == '\n') {
			if (*len > 0U) {
				buf[*len] = '\0';
				debug_send_rx_line_tagged(tag, buf, *len);
				at_handler_process_line_from_uart(src_uart, buf, *len);
				at_handler_run_deferred_action();
				uart_rx_reset(buf, len);
			}
			continue;
		}

		if (!uart_rx_is_printable(ch)) {
			if (g_uart_debug_logging_enabled) {
				debug_send_label("RX_DROP_NONPRINTABLE:0x");
				uart_send_hex8(ch);
				uart_send_str_all("\r\n");
			}
			uart_rx_reset(buf, len);
			continue;
		}

		if (*len < (CMD_BUF_SIZE - 1U)) {
			buf[(*len)++] = (char)ch;
		} else {
			uart_rx_reset(buf, len);
			debug_send_u32("RX_BUFFER_OVERFLOW:len=", CMD_BUF_SIZE - 1U);
			uart_send_line("ERROR:BUFFER_OVERFLOW");
		}
	}

	if (*len > 0U && *last_rx_ms > 0 &&
	    (k_uptime_get() - *last_rx_ms) >= CMD_IDLE_FLUSH_MS) {
		buf[*len] = '\0';
		if (g_uart_debug_logging_enabled) {
			debug_send_label("RX_IDLE_FLUSH:");
			uart_send_str_all(tag);
			uart_send_str_all("\r\n");
		}
		debug_send_rx_line_tagged(tag, buf, *len);
		at_handler_process_line_from_uart(src_uart, buf, *len);
		at_handler_run_deferred_action();
		uart_rx_reset(buf, len);
		*last_rx_ms = 0;
	}
}

static bool uart_rx_is_printable(uint8_t ch)
{
	return ch >= 0x20U && ch <= 0x7eU;
}

static void uart_rx_reset(char *buf, size_t *len)
{
	if (buf != NULL) {
		buf[0] = '\0';
	}

	if (len != NULL) {
		*len = 0U;
	}
}

static void update_keywake_boot_state(struct factory_persist *persist)
{
#if defined(CONFIG_HWINFO)
	uint32_t reset_cause = 0U;
	int rc;

	if (persist == NULL) {
		return;
	}

	rc = hwinfo_get_reset_cause(&reset_cause);
	if (rc != 0) {
		return;
	}

	if ((persist->reserved[FACTORY_PERSIST_FLAGS_IDX] &
	     FACTORY_PERSIST_FLAG_SLEEPI_ARMED) == 0U) {
		return;
	}

	persist->reserved[FACTORY_PERSIST_FLAGS_IDX] &=
		~FACTORY_PERSIST_FLAG_SLEEPI_ARMED;
	persist->reserved[FACTORY_PERSIST_FLAGS_IDX] &=
		~(FACTORY_PERSIST_FLAG_KEYWAKE_LATCHED |
		  FACTORY_PERSIST_FLAG_KEYWAKE_SW0);
	persist->reserved[FACTORY_PERSIST_RESET_CAUSE_IDX] = reset_cause;

	if ((reset_cause & RESET_LOW_POWER_WAKE) != 0U) {
		persist->reserved[FACTORY_PERSIST_FLAGS_IDX] |=
			FACTORY_PERSIST_FLAG_KEYWAKE_LATCHED |
			FACTORY_PERSIST_FLAG_KEYWAKE_SW0;
		persist->reserved[FACTORY_PERSIST_WAKE_COUNT_IDX]++;
	}

	(void)factory_storage_save(persist);
#else
	ARG_UNUSED(persist);
#endif
}

static void run_factory_program(void)
{
	uart_send_line("=== XIAO nRF54LM20A Factory UART V3 ===");
	uart_send_line("UART: UART21 @ 115200");
	uart_send_line("Use text commands or legacy AT+HELP");
	uart_send_line("====================================");
}

int main(void)
{
	char cmd_buf_uart21[CMD_BUF_SIZE];
	char cmd_buf_uart20[CMD_BUF_SIZE];
	size_t cmd_len_uart21 = 0;
	size_t cmd_len_uart20 = 0;
	int64_t last_rx_ms_uart21 = 0;
	int64_t last_rx_ms_uart20 = 0;
	int rc;

	uart_rx_reset(cmd_buf_uart21, &cmd_len_uart21);
	uart_rx_reset(cmd_buf_uart20, &cmd_len_uart20);
	g_uart_debug_logging_enabled = false;
	g_dual_uart_diag_enabled = false;

	if (!device_is_ready(uart_dev)) {
		return 0;
	}

	if (g_dual_uart_diag_enabled &&
	    uart20_dev != NULL && device_is_ready(uart20_dev)) {
		const struct uart_config debug_uart20_cfg = {
			.baudrate = 115200,
			.parity = UART_CFG_PARITY_NONE,
			.stop_bits = UART_CFG_STOP_BITS_1,
			.data_bits = UART_CFG_DATA_BITS_8,
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		};

		(void)uart_configure(uart20_dev, &debug_uart20_cfg);
	}

	g_led_ready = false;
	if (gpio_is_ready_dt(&led)) {
		rc = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
		if (rc == 0) {
			g_led_ready = true;
		}
	}

	rc = factory_storage_load(&g_persist);
	if (rc != 0) {
		factory_storage_defaults(&g_persist);
		(void)factory_storage_save(&g_persist);
	}

	/* In factory mode: turn off R/G LEDs, only BLUE blinks.
	 * Otherwise (pre-factory): set R/G ON for solid white LED.
	 * LEDs are active-low on XIAO nRF54LM20A:
	 *   GPIO_OUTPUT_LOW  = ON,  GPIO_OUTPUT_HIGH = OFF
	 * (DTS says GPIO_ACTIVE_HIGH but hardware is inverted).
	 */
	if (g_persist.boot_flag == FACTORY_BOOT_FLAG_ENTER_FACTORY) {
#if DT_NODE_EXISTS(DT_ALIAS(led1))
		if (gpio_is_ready_dt(&led1)) {
			(void)gpio_pin_configure(led1.port, led1.pin,
						 GPIO_OUTPUT_HIGH);
		}
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
		if (gpio_is_ready_dt(&led2)) {
			(void)gpio_pin_configure(led2.port, led2.pin,
						 GPIO_OUTPUT_HIGH);
		}
#endif
	} else {
#if DT_NODE_EXISTS(DT_ALIAS(led1))
		if (gpio_is_ready_dt(&led1)) {
			(void)gpio_pin_configure(led1.port, led1.pin,
						 GPIO_OUTPUT_LOW);
		}
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
		if (gpio_is_ready_dt(&led2)) {
			(void)gpio_pin_configure(led2.port, led2.pin,
						 GPIO_OUTPUT_LOW);
		}
#endif
	}

	debug_send_u32("BOOT:boot_flag=", g_persist.boot_flag);
	debug_send_u32("BOOT:led_ready=", g_led_ready ? 1U : 0U);

	/* Detect sleep mode wakeup — print log and clear flag */
	if ((g_persist.reserved[FACTORY_PERSIST_FLAGS_IDX] &
	     FACTORY_PERSIST_FLAG_SLEEP_WAKE) != 0U) {
		uart_send_line("BOOT: sleep mode wakeup detected");
		g_persist.reserved[FACTORY_PERSIST_FLAGS_IDX] &=
			~FACTORY_PERSIST_FLAG_SLEEP_WAKE;
		(void)factory_storage_save(&g_persist);
	}

	update_keywake_boot_state(&g_persist);

	/* Detect ship mode recovery — clear flag so next boot is clean */
	if ((g_persist.reserved[FACTORY_PERSIST_FLAGS_IDX] &
	     FACTORY_PERSIST_FLAG_SHIPMODE_ARMED) != 0U) {
		g_persist.reserved[FACTORY_PERSIST_FLAGS_IDX] &=
			~FACTORY_PERSIST_FLAG_SHIPMODE_ARMED;
		(void)factory_storage_save(&g_persist);
	}

	if (g_persist.boot_flag == FACTORY_BOOT_FLAG_ENTER_FACTORY) {
		debug_send_line("BOOT:path=", "factory_program");
		run_factory_program();
	} else {
		debug_send_line("BOOT:path=", "command_loop");

		uart_send_line("=== XIAO nRF54LM20A Factory UART V3 ===");
		uart_send_line("UART: UART21 @ 115200");
		uart_send_line("Use text commands or legacy AT+HELP");
		uart_send_line("====================================");
	}

	at_handler_init(uart_dev, regulator_parent, &g_persist);
	at_handler_early_init();

	bool factory_blink_on = false;

	while (1) {
		bool handled_uart = false;

		process_uart_rx(uart_dev, "RX21_LINE:", cmd_buf_uart21,
			       &cmd_len_uart21, &last_rx_ms_uart21, &handled_uart);

		if (g_dual_uart_diag_enabled && !at_handler_uart20_service_enabled()) {
			process_uart_rx(uart20_dev, "RX20_LINE:", cmd_buf_uart20,
				       &cmd_len_uart20, &last_rx_ms_uart20,
				       &handled_uart);
		}

		at_handler_poll_background();

		if (!handled_uart) {
			k_usleep(100);
		}

		/* Factory mode: blink LED and print status every 1 second */
		if (g_persist.boot_flag == FACTORY_BOOT_FLAG_ENTER_FACTORY) {
			static int64_t last_blink_ms;
			static bool factory_rg_off;

			/* Turn off R/G on first entry (handles runtime
			 * transition from non-factory via "flash 2" command).
			 */
			if (!factory_rg_off) {
				factory_rg_off = true;
#if DT_NODE_EXISTS(DT_ALIAS(led1))
				if (gpio_is_ready_dt(&led1)) {
					(void)gpio_pin_configure(led1.port,
						led1.pin, GPIO_OUTPUT_HIGH);
				}
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
				if (gpio_is_ready_dt(&led2)) {
					(void)gpio_pin_configure(led2.port,
						led2.pin, GPIO_OUTPUT_HIGH);
				}
#endif
			}

			if (last_blink_ms == 0) {
				last_blink_ms = k_uptime_get();
			}

			if ((k_uptime_get() - last_blink_ms) >= 1000) {
				last_blink_ms = k_uptime_get();
				factory_blink_on = !factory_blink_on;

				if (g_led_ready) {
					(void)gpio_pin_set_dt(&led,
							      factory_blink_on ? 1 : 0);
				}
				uart_send_str_dev(uart20_dev, factory_blink_on
					       ? "XIAO nRF54LM20A Demo, LED ON\r\n"
					       : "XIAO nRF54LM20A Demo, LED OFF\r\n");
			}
		}
	}

	return 0;
}
