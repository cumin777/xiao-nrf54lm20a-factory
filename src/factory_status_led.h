#ifndef FACTORY_STATUS_LED_H_
#define FACTORY_STATUS_LED_H_

#include <stdint.h>

void factory_status_led_init(void);
void factory_status_led_update(uint32_t boot_flag);

#endif /* FACTORY_STATUS_LED_H_ */
