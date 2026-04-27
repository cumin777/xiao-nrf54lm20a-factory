#ifndef FACTORY_STORAGE_H_
#define FACTORY_STORAGE_H_

#include <stdint.h>

#define FACTORY_STORAGE_MAGIC 0xFA51B007u
#define FACTORY_STORAGE_VERSION 2u
#define FACTORY_BOOT_FLAG_ENTER_FACTORY 2u

#define FACTORY_PERSIST_FLAGS_IDX 0u
#define FACTORY_PERSIST_WAKE_COUNT_IDX 1u
#define FACTORY_PERSIST_RESET_CAUSE_IDX 2u

#define FACTORY_PERSIST_FLAG_SLEEPI_ARMED (1u << 0)
#define FACTORY_PERSIST_FLAG_KEYWAKE_LATCHED (1u << 1)
#define FACTORY_PERSIST_FLAG_KEYWAKE_SW0 (1u << 2)
#define FACTORY_PERSIST_FLAG_SHIPMODE_ARMED (1u << 3)

struct factory_persist {
	uint32_t magic;
	uint32_t version;
	uint32_t boot_flag;
	uint32_t state_bitmap;
	uint32_t item_bitmap;
	uint32_t reserved[3];
	uint32_t legacy_flash_value;
};

void factory_storage_defaults(struct factory_persist *data);
int factory_storage_load(struct factory_persist *out);
int factory_storage_save(const struct factory_persist *in);

#endif /* FACTORY_STORAGE_H_ */
