#include "factory_storage.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>

#define FACTORY_STORAGE_AREA_ID FIXED_PARTITION_ID(storage_partition)

struct factory_persist_v1 {
	uint32_t magic;
	uint32_t version;
	uint32_t boot_flag;
	uint32_t state_bitmap;
	uint32_t item_bitmap;
	uint32_t reserved[3];
};

static bool factory_storage_is_valid(const struct factory_persist *data)
{
	return data->magic == FACTORY_STORAGE_MAGIC &&
	       data->version == FACTORY_STORAGE_VERSION;
}

static bool factory_storage_is_valid_v1(const struct factory_persist_v1 *data)
{
	return data->magic == FACTORY_STORAGE_MAGIC && data->version == 1u;
}

static void factory_storage_migrate_v1(const struct factory_persist_v1 *old_data,
				       struct factory_persist *out)
{
	factory_storage_defaults(out);
	out->boot_flag = old_data->boot_flag;
	out->state_bitmap = old_data->state_bitmap;
	out->item_bitmap = old_data->item_bitmap;
	memcpy(out->reserved, old_data->reserved, sizeof(old_data->reserved));
	out->legacy_flash_value = 0u;
}

void factory_storage_defaults(struct factory_persist *data)
{
	memset(data, 0, sizeof(*data));
	data->magic = FACTORY_STORAGE_MAGIC;
	data->version = FACTORY_STORAGE_VERSION;
}

int factory_storage_load(struct factory_persist *out)
{
	const struct flash_area *fa;
	struct factory_persist tmp;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}

	rc = flash_area_open(FACTORY_STORAGE_AREA_ID, &fa);
	if (rc != 0) {
		return rc;
	}

	if (fa->fa_size < sizeof(tmp)) {
		flash_area_close(fa);
		return -ENOSPC;
	}

	rc = flash_area_read(fa, 0, &tmp, sizeof(tmp));
	flash_area_close(fa);
	if (rc != 0) {
		return rc;
	}

	if (!factory_storage_is_valid(&tmp)) {
		const struct factory_persist_v1 *old_data =
			(const struct factory_persist_v1 *)&tmp;

		if (factory_storage_is_valid_v1(old_data)) {
			factory_storage_migrate_v1(old_data, out);
			return 0;
		}

		factory_storage_defaults(out);
		return -ENODATA;
	}

	*out = tmp;
	return 0;
}

int factory_storage_save(const struct factory_persist *in)
{
	const struct flash_area *fa;
	struct factory_persist tmp;
	int rc;

	if (in == NULL) {
		return -EINVAL;
	}

	tmp = *in;
	tmp.magic = FACTORY_STORAGE_MAGIC;
	tmp.version = FACTORY_STORAGE_VERSION;

	rc = flash_area_open(FACTORY_STORAGE_AREA_ID, &fa);
	if (rc != 0) {
		return rc;
	}

	if (fa->fa_size < sizeof(tmp)) {
		flash_area_close(fa);
		return -ENOSPC;
	}

	rc = flash_area_erase(fa, 0, fa->fa_size);
	if (rc == 0) {
		rc = flash_area_write(fa, 0, &tmp, sizeof(tmp));
	}

	flash_area_close(fa);
	return rc;
}
