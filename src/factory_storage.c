#include "factory_storage.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>

#define FACTORY_STORAGE_AREA_ID FIXED_PARTITION_ID(storage_partition)

static bool factory_storage_is_valid(const struct factory_persist *data)
{
	return data->magic == FACTORY_STORAGE_MAGIC &&
	       data->version == FACTORY_STORAGE_VERSION;
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
