#ifndef AT_HANDLER_H_
#define AT_HANDLER_H_

#include <stddef.h>
#include <zephyr/device.h>

#include "factory_storage.h"

void at_handler_init(const struct device *uart_dev,
		     const struct device *regulator_parent,
		     struct factory_persist *persist);

void at_handler_process_line_from_uart(const struct device *uart_dev,
				       const char *line, size_t len);
bool at_handler_uart20_service_enabled(void);
void at_handler_poll_background(void);
void at_handler_run_deferred_action(void);

#endif /* AT_HANDLER_H_ */
