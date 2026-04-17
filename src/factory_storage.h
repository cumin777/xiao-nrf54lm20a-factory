#ifndef FACTORY_STORAGE_H_
#define FACTORY_STORAGE_H_

#include <stdint.h>

#define FACTORY_STORAGE_MAGIC 0xFA51B007u
#define FACTORY_STORAGE_VERSION 1u
#define FACTORY_BOOT_FLAG_ENTER_FACTORY 2u

struct factory_persist {
	uint32_t magic;
	uint32_t version;
	uint32_t boot_flag;
	uint32_t state_bitmap;
	uint32_t item_bitmap;
	uint32_t reserved[3];
};

void factory_storage_defaults(struct factory_persist *data);
int factory_storage_load(struct factory_persist *out);
int factory_storage_save(const struct factory_persist *in);

#endif /* FACTORY_STORAGE_H_ */
