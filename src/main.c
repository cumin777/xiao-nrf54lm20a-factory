#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "at_handler.h"
#include "factory_storage.h"

#define UART_NODE DT_NODELABEL(uart21)
#define LED_NODE DT_ALIAS(led0)
#define REGULATOR_PARENT_NODE DT_ALIAS(regulator_parent)

#define AT_BUF_SIZE 160

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct device *const regulator_parent =
	DEVICE_DT_GET_OR_NULL(REGULATOR_PARENT_NODE);

static struct factory_persist g_persist;

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

static void run_factory_program(void)
{
	const struct uart_config factory_uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	(void)uart_configure(uart_dev, &factory_uart_cfg);

	while (1) {
		gpio_pin_set_dt(&led, 1);
		uart_send_line("XIAO nRF54LM20A Demo, LED ON");
		k_msleep(1000);

		gpio_pin_set_dt(&led, 0);
		uart_send_line("XIAO nRF54LM20A Demo, LED OFF");
		k_msleep(1000);
	}
}

int main(void)
{
	char at_buf[AT_BUF_SIZE];
	size_t at_len = 0;
	int rc;

	if (!device_is_ready(uart_dev)) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	rc = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		return 0;
	}

	rc = factory_storage_load(&g_persist);
	if (rc != 0) {
		factory_storage_defaults(&g_persist);
		(void)factory_storage_save(&g_persist);
	}

	if (g_persist.boot_flag == FACTORY_BOOT_FLAG_ENTER_FACTORY) {
		run_factory_program();
	}

	at_handler_init(uart_dev, regulator_parent, &g_persist);

	uart_send_line("=== XIAO nRF54LM20A Factory AT V2 ===");
	uart_send_line("UART: UART21 @ 115200");
	uart_send_line("Use AT+HELP for command list");
	uart_send_line("====================================");

	while (1) {
		uint8_t ch;

		if (uart_poll_in(uart_dev, &ch) == 0) {
			if (ch == '\r' || ch == '\n') {
				if (at_len > 0) {
					at_handler_process_line(at_buf, at_len);
					at_len = 0;
				}
			} else if (at_len < (AT_BUF_SIZE - 1U)) {
				at_buf[at_len++] = (char)ch;
			} else {
				at_len = 0;
				uart_send_line("ERROR:BUFFER_OVERFLOW");
			}
		} else {
			k_usleep(100);
		}
	}

	return 0;
}
