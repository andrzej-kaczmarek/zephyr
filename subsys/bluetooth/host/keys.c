/* keys.c - Bluetooth key handling */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <atomic.h>
#include <misc/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>
#include <bluetooth/storage.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_KEYS)
#include "common/log.h"

#include "common/rpa.h"
#include "hci_core.h"
#include "smp.h"
#include "keys.h"

static struct bt_keys key_pool[CONFIG_BT_MAX_PAIRED];

struct bt_keys *bt_keys_get_addr(const bt_addr_le_t *addr)
{
	struct bt_keys *keys;
	int i;

	BT_DBG("%s", bt_addr_le_str(addr));

	for (i = 0; i < ARRAY_SIZE(key_pool); i++) {
		keys = &key_pool[i];

		if (!bt_addr_le_cmp(&keys->addr, addr)) {
			return keys;
		}

		if (!bt_addr_le_cmp(&keys->addr, BT_ADDR_LE_ANY)) {
			bt_addr_le_copy(&keys->addr, addr);
			BT_DBG("created %p for %s", keys, bt_addr_le_str(addr));
			return keys;
		}
	}

	BT_DBG("unable to create keys for %s", bt_addr_le_str(addr));

	return NULL;
}
struct bt_keys *bt_keys_find(int type, const bt_addr_le_t *addr)
{
	int i;

	BT_DBG("type %d %s", type, bt_addr_le_str(addr));

	for (i = 0; i < ARRAY_SIZE(key_pool); i++) {
		if ((key_pool[i].keys & type) &&
		    !bt_addr_le_cmp(&key_pool[i].addr, addr)) {
			return &key_pool[i];
		}
	}

	return NULL;
}

struct bt_keys *bt_keys_get_type(int type, const bt_addr_le_t *addr)
{
	struct bt_keys *keys;

	BT_DBG("type %d %s", type, bt_addr_le_str(addr));

	keys = bt_keys_find(type, addr);
	if (keys) {
		return keys;
	}

	keys = bt_keys_get_addr(addr);
	if (!keys) {
		return NULL;
	}

	bt_keys_add_type(keys, type);

	return keys;
}

struct bt_keys *bt_keys_find_irk(const bt_addr_le_t *addr)
{
	int i;

	BT_DBG("%s", bt_addr_le_str(addr));

	if (!bt_addr_le_is_rpa(addr)) {
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(key_pool); i++) {
		if (!(key_pool[i].keys & BT_KEYS_IRK)) {
			continue;
		}

		if (!bt_addr_cmp(&addr->a, &key_pool[i].irk.rpa)) {
			BT_DBG("cached RPA %s for %s",
			       bt_addr_str(&key_pool[i].irk.rpa),
			       bt_addr_le_str(&key_pool[i].addr));
			return &key_pool[i];
		}
	}

	for (i = 0; i < ARRAY_SIZE(key_pool); i++) {
		if (!(key_pool[i].keys & BT_KEYS_IRK)) {
			continue;
		}

		if (bt_rpa_irk_matches(key_pool[i].irk.val, &addr->a)) {
			BT_DBG("RPA %s matches %s",
			       bt_addr_str(&key_pool[i].irk.rpa),
			       bt_addr_le_str(&key_pool[i].addr));

			bt_addr_copy(&key_pool[i].irk.rpa, &addr->a);

			return &key_pool[i];
		}
	}

	BT_DBG("No IRK for %s", bt_addr_le_str(addr));

	return NULL;
}

struct bt_keys *bt_keys_find_addr(const bt_addr_le_t *addr)
{
	int i;

	BT_DBG("%s", bt_addr_le_str(addr));

	for (i = 0; i < ARRAY_SIZE(key_pool); i++) {
		if (!bt_addr_le_cmp(&key_pool[i].addr, addr)) {
			return &key_pool[i];
		}
	}

	return NULL;
}

void bt_keys_add_type(struct bt_keys *keys, int type)
{
	keys->keys |= type;
}

void bt_keys_clear(struct bt_keys *keys)
{
	BT_DBG("keys for %s", bt_addr_le_str(&keys->addr));

	if (bt_storage) {
		bt_addr_le_t addrs[CONFIG_BT_MAX_PAIRED + 1];
		bool found = false;
		int i;

		memset(addrs, 0, sizeof(addrs));

		bt_storage->clear(&keys->addr);

		bt_storage->read(NULL, BT_STORAGE_ADDRESSES, addrs,
				 sizeof(bt_addr_le_t) * CONFIG_BT_MAX_PAIRED);

		for (i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
			if (!bt_addr_le_cmp(&addrs[i], BT_ADDR_LE_ANY)) {
				/* end of list */
				break;
			}

			if (!bt_addr_le_cmp(&addrs[i], &keys->addr)) {
				found = true;
			}

			if (found) {
				bt_addr_le_copy(&addrs[i], &addrs[i + 1]);
			}
		}

		if (found) {
			BT_DBG("addresses updated");
			bt_storage->write(NULL, BT_STORAGE_ADDRESSES, addrs,
					  sizeof(bt_addr_le_t) *
					  CONFIG_BT_MAX_PAIRED);
		}
	}

	memset(keys, 0, sizeof(*keys));
}

void bt_keys_clear_all(void)
{
	memset(key_pool, 0, sizeof(key_pool));

	bt_storage->clear(BT_ADDR_LE_ANY);
	BT_DBG("addresses updated");
	bt_storage->write(NULL, BT_STORAGE_ADDRESSES, BT_ADDR_LE_ANY,
			  sizeof(bt_addr_le_t));
}

void bt_keys_persist(struct bt_keys *keys)
{
	bt_addr_le_t addrs[CONFIG_BT_MAX_PAIRED];
	struct bt_storage_ltk ltk;
#if defined(CONFIG_BT_SIGNING)
	struct bt_storage_csrk csrk;
#endif
	bool found = false;
	int i;

	BT_DBG("keys for %s", bt_addr_le_str(&keys->addr));

	if (!bt_storage) {
		return;
	}

	memset(&ltk, 0, sizeof(ltk));
#if defined(CONFIG_BT_SIGNING)
	memset(&csrk, 0, sizeof(csrk));
#endif

	if (atomic_test_bit(keys->flags, BT_KEYS_AUTHENTICATED)) {
		BT_DBG("auth");
		ltk.flags |= BT_STORAGE_LTK_AUTHENTICATED;
#if defined(CONFIG_BT_SIGNING)
		csrk.flags |= BT_STORAGE_CSRK_AUTHENTICATED;
#endif
	}

	bt_storage->clear(&keys->addr);

	if (keys->keys & BT_KEYS_IRK) {
		BT_DBG("IRK");
		bt_storage->write(&keys->addr, BT_STORAGE_IRK, &keys->irk.val,
				  16);
	}

	if (keys->keys & BT_KEYS_LTK_P256) {
		BT_DBG("LTK P256");
		ltk.flags = BT_STORAGE_LTK_SC;
		ltk.size = keys->enc_size;
		ltk.ediv = keys->ltk.ediv;
		memcpy(ltk.rand, &keys->ltk.rand, 8);
		memcpy(ltk.val, keys->ltk.val, 16);
		bt_storage->write(&keys->addr, BT_STORAGE_LTK,
				  &ltk, sizeof(ltk));
	}

	if (keys->keys & BT_KEYS_LTK) {
		BT_DBG("LTK");
		ltk.flags = 0;
		ltk.size = keys->enc_size;
		ltk.ediv = keys->ltk.ediv;
		memcpy(ltk.rand, &keys->ltk.rand, 8);
		memcpy(ltk.val, keys->ltk.val, 16);
		bt_storage->write(&keys->addr, BT_STORAGE_LTK,
				  &ltk, sizeof(ltk));
	}

#if !defined(CONFIG_BT_SMP_SC_ONLY)
	if (keys->keys & BT_KEYS_SLAVE_LTK) {
		BT_DBG("Slave LTK");
		ltk.size = keys->enc_size;
		ltk.ediv = keys->slave_ltk.ediv;
		memcpy(ltk.rand, &keys->slave_ltk.rand, 8);
		memcpy(ltk.val, keys->slave_ltk.val, 16);
		bt_storage->write(&keys->addr, BT_STORAGE_SLAVE_LTK,
				  &ltk, sizeof(ltk));
	}
#endif

#if defined(CONFIG_BT_SIGNING)
	if (keys->keys & BT_KEYS_LOCAL_CSRK) {
		BT_DBG("Local CSRK");
		csrk.counter = keys->local_csrk.cnt;
		memcpy(csrk.val, keys->local_csrk.val, 16);
		bt_storage->write(&keys->addr, BT_STORAGE_LOCAL_CSRK,
				  &csrk, sizeof(csrk));
	}

	if (keys->keys & BT_KEYS_REMOTE_CSRK) {
		BT_DBG("Remote CSRK");
		csrk.counter = keys->remote_csrk.cnt;
		memcpy(csrk.val, keys->remote_csrk.val, 16);
		bt_storage->write(&keys->addr, BT_STORAGE_REMOTE_CSRK,
				  &csrk, sizeof(csrk));
	}
#endif

	memset(addrs, 0, sizeof(addrs));

	bt_storage->read(NULL, BT_STORAGE_ADDRESSES, addrs, sizeof(addrs));

	for (i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
		if (!bt_addr_le_cmp(&addrs[i], &keys->addr)) {
			found = true;
			break;
		}

		if (!bt_addr_le_cmp(&addrs[i], BT_ADDR_LE_ANY)) {
			bt_addr_le_copy(&addrs[i], &keys->addr);
			if (i + 1 < CONFIG_BT_MAX_PAIRED) {
				/* null terminate */
				bt_addr_le_copy(&addrs[i + 1], BT_ADDR_LE_ANY);
			}
			break;
		}
	}

	if (!found) {
		BT_DBG("addresses updated");
		bt_storage->write(NULL, BT_STORAGE_ADDRESSES, addrs,
				  sizeof(bt_addr_le_t) * CONFIG_BT_MAX_PAIRED);
	}
}

void bt_keys_restore(void)
{
	bt_addr_le_t addrs[CONFIG_BT_MAX_PAIRED];
	struct bt_storage_ltk ltk;
#if defined(CONFIG_BT_SIGNING)
	struct bt_storage_csrk csrk;
#endif
	u8_t irk[16];
	struct bt_keys *keys;
	ssize_t read;
	int i;

	BT_DBG("");

	if (!bt_storage) {
		return;
	}

	memset(addrs, 0, sizeof(addrs));

	bt_storage->read(NULL, BT_STORAGE_ADDRESSES, addrs, sizeof(addrs));

	for (i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
		if (!bt_addr_le_cmp(&addrs[i], BT_ADDR_LE_ANY)) {
			/* end of list */
			break;
		}

		BT_DBG("keys for %s", bt_addr_le_str(&addrs[i]));

		keys = bt_keys_get_addr(&addrs[i]);
		if (!keys) {
			continue;
		}

		read = bt_storage->read(&keys->addr, BT_STORAGE_IRK, irk,
					sizeof(irk));
		if (read == sizeof(irk)) {
			BT_DBG("IRK");
			bt_keys_add_type(keys, BT_KEYS_IRK);
			memcpy(keys->irk.val, irk, 16);
		}

		read = bt_storage->read(&keys->addr, BT_STORAGE_LTK, &ltk,
					sizeof(ltk));
		if (read == sizeof(ltk)) {
			if (ltk.flags & BT_STORAGE_LTK_SC) {
				BT_DBG("LTK P256");
				bt_keys_add_type(keys, BT_KEYS_LTK_P256);
			} else {
				BT_DBG("LTK");
				bt_keys_add_type(keys, BT_KEYS_LTK);
			}
			keys->enc_size = ltk.size;
			keys->ltk.ediv = ltk.ediv;
			memcpy(&keys->ltk.rand, ltk.rand, 8);
			memcpy(keys->ltk.val, ltk.val, 16);
		}

#if !defined(CONFIG_BT_SMP_SC_ONLY)
		read = bt_storage->read(&keys->addr, BT_STORAGE_SLAVE_LTK, &ltk,
					sizeof(ltk));
		if (read == sizeof(ltk)) {
			BT_DBG("Slave LTK");
			bt_keys_add_type(keys, BT_KEYS_SLAVE_LTK);
			keys->enc_size = ltk.size;
			keys->slave_ltk.ediv = ltk.ediv;
			memcpy(&keys->slave_ltk.rand, ltk.rand, 8);
			memcpy(keys->slave_ltk.val, ltk.val, 16);
		}
#endif

#if defined(CONFIG_BT_SIGNING)
		read = bt_storage->read(&keys->addr, BT_STORAGE_LOCAL_CSRK,
					&csrk, sizeof(csrk));
		if (read == sizeof(csrk)) {
			BT_DBG("Local CSRK");
			bt_keys_add_type(keys, BT_KEYS_LOCAL_CSRK);
			keys->local_csrk.cnt = csrk.counter;
			memcpy(keys->local_csrk.val, csrk.val, 16);
		}

		read = bt_storage->read(&keys->addr, BT_STORAGE_REMOTE_CSRK,
					&csrk, sizeof(csrk));
		if (read == sizeof(csrk)) {
			BT_DBG("Remote CSRK");
			bt_keys_add_type(keys, BT_KEYS_REMOTE_CSRK);
			keys->remote_csrk.cnt = csrk.counter;
			memcpy(keys->remote_csrk.val, csrk.val, 16);
		}
#endif
	}
}
