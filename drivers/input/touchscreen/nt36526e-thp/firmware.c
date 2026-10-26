// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * Adapted from MiCode piano p81/nt36532/nt36xxx_fw_update.c and nt36xxx.c.
 * See README.rst for pinned sources and the supported firmware layout.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "firmware.h"

#define NVT_ADDRESS_MAX		0x7fffff
#define NVT_AUTO_COPY		0x1fc825
#define NVT_WRITE_CHUNK		0x3c00
#define NVT_FW_STATE		0x130960
#define NVT_FWINFO		0x130978
#define NVT_RESET		0x1fb43e
#define NVT_BOOT_READY		0x1fb50d

static int nvt_partition(const u8 *data, size_t size, size_t position,
			 bool primary, struct nvt_thp_partition *part)
{
	u32 encoded_size;

	if (position > size || size - position < (primary ? 12 : 16))
		return -EINVAL;
	if (primary) {
		part->offset = get_unaligned_le32(data + position);
		part->address = get_unaligned_le32(data + position + 4);
		encoded_size = get_unaligned_le32(data + position + 8);
		part->crc = get_unaligned_le32(data + 0x18 + position / 12 * 4);
	} else {
		part->address = get_unaligned_le32(data + position);
		encoded_size = get_unaligned_le32(data + position + 4);
		part->offset = get_unaligned_le32(data + position + 8);
		part->crc = get_unaligned_le32(data + position + 12);
	}
	/* Zero denotes a reserved entry; all other lengths are inclusive. */
	if (!encoded_size) {
		part->length = 0;
		return primary ? -EINVAL : 0;
	}
	if (part->offset >= size || encoded_size >= size - part->offset ||
	    part->address > NVT_ADDRESS_MAX || encoded_size > NVT_ADDRESS_MAX - part->address)
		return -EINVAL;
	part->length = encoded_size + 1;
	return 0;
}

int nvt_thp_parse_firmware(const u8 *data, size_t size,
			   struct nvt_thp_firmware *image)
{
	size_t end, first_end, position, write_end;
	unsigned int info_count, overlays, index;
	bool found_header = false;
	int ret;

	memset(image, 0, sizeof(*image));
	if (size < SZ_4K)
		return -EINVAL;
	for (write_end = round_down(size, SZ_4K); write_end; write_end -= SZ_4K)
		if (!memcmp(data + write_end - 3, "NVT", 3) ||
		    !memcmp(data + write_end - 3, "MOD", 3))
			break;
	if (!write_end || data[write_end - SZ_4K] + data[write_end - SZ_4K + 1] != 0xff)
		return -ENOEXEC;

	end = get_unaligned_le32(data);
	if (end < 0x40 || end > size || !IS_ALIGNED(end, 16))
		return -EINVAL;
	/* This transport supports the observed cascade, non-DMA-divider layout. */
	if (!(data[0x20] & BIT(1)) || (data[0x29] & BIT(0)))
		return -EOPNOTSUPP;
	first_end = end;
	for (position = 0x30; position < end; position += 16) {
		if (get_unaligned_le32(data + position + 4) &&
		    !get_unaligned_le32(data + position + 8)) {
			first_end = position + 16;
			found_header = true;
			break;
		}
	}
	if (!found_header || first_end > end / 2)
		return -EINVAL;
	end = first_end * 2;
	info_count = (first_end - 0x30) / 16 + 1;
	overlays = data[0x28] & BIT(4) ? data[0x28] & 0x0f : 0;
	if (info_count > U8_MAX)
		return -E2BIG;
	image->count = 2 + info_count + overlays;
	image->partitions = kcalloc(image->count, sizeof(*image->partitions), GFP_KERNEL);
	if (!image->partitions)
		return -ENOMEM;
	found_header = false;
	for (index = 0; index < image->count; ++index) {
		if (index < 2)
			position = index * 12;
		else if (index < 2 + info_count)
			position = found_header ? end - 16 : 0x30 + (index - 2) * 16;
		else
			position = (size_t)image->partitions[1].offset +
				   (index - 2 - info_count) * 16;
		ret = nvt_partition(data, size, position, index < 2, &image->partitions[index]);
		if (ret)
			goto fail;
		if (index >= 2 && index < 2 + info_count &&
		    image->partitions[index].length && image->partitions[index].offset < end)
			found_header = true;
	}
	return 0;
fail:
	nvt_thp_free_firmware(image);
	return ret;
}

void nvt_thp_free_firmware(struct nvt_thp_firmware *image)
{
	kfree(image->partitions);
	memset(image, 0, sizeof(*image));
}

/*
 * Vendor nvt_cmd_store(): host command register is at client_base + 0x50.
 * Write {cmd, val}, then poll until the first data byte reads 0.
 */
#define NVT_HOST_CMD	0x130950

static int nvt_host_cmd(struct regmap *map, u8 cmd, u8 val)
{
	u8 buf[2] = { cmd, val };
	u8 st[2];
	int i, ret;

	ret = regmap_bulk_write(map, NVT_HOST_CMD, buf, sizeof(buf));
	if (ret)
		return ret;
	for (i = 0; i < 100; i++) {
		usleep_range(1000, 1500);
		ret = regmap_bulk_read(map, NVT_HOST_CMD, st, sizeof(st));
		if (ret)
			return ret;
		if (st[0] == 0)
			return 0;
	}
	return -ETIMEDOUT;
}

/* Vendor nvt_edge_reject_set(0) == host command 0xba (single byte). */
static int nvt_host_cmd1(struct regmap *map, u8 cmd)
{
	u8 st[2];
	int i, ret;

	ret = regmap_bulk_write(map, NVT_HOST_CMD, &cmd, 1);
	if (ret)
		return ret;
	for (i = 0; i < 20; i++) {
		usleep_range(1000, 1500);
		ret = regmap_bulk_read(map, NVT_HOST_CMD, st, sizeof(st));
		if (ret)
			return ret;
		if (st[0] == 0)
			return 0;
	}
	return -ETIMEDOUT;
}

/* Host command with two payload bytes (vendor sends 0x7b 04 01 at probe). */
static int nvt_host_cmd2(struct regmap *map, u8 cmd, u8 v1, u8 v2)
{
	u8 buf[3] = { cmd, v1, v2 };
	u8 st[2];
	int i, ret;

	ret = regmap_bulk_write(map, NVT_HOST_CMD, buf, sizeof(buf));
	if (ret)
		return ret;
	for (i = 0; i < 100; i++) {
		usleep_range(1000, 1500);
		ret = regmap_bulk_read(map, NVT_HOST_CMD, st, sizeof(st));
		if (ret)
			return ret;
		if (st[0] == 0)
			return 0;
	}
	return -ETIMEDOUT;
}

static int nvt_boot_reset(struct regmap *map)
{
	int ret = regmap_write(map, NVT_RESET, 0x69);

	if (!ret)
		usleep_range(5000, 6000); /* Vendor tBRST2FR. */
	return ret;
}

static int nvt_crc_bank(struct regmap *map, const struct nvt_thp_partition *part,
			u32 destination, u32 length, u32 golden)
{
	u8 bytes[4];
	int ret;

	put_unaligned_le32(part->address, bytes);
	ret = regmap_bulk_write(map, destination, bytes, 3);
	if (ret)
		return ret;
	put_unaligned_le32(part->length - 1, bytes);
	ret = regmap_bulk_write(map, length, bytes, 3);
	if (ret)
		return ret;
	put_unaligned_le32(part->crc, bytes);
	return regmap_bulk_write(map, golden, bytes, sizeof(bytes));
}

static int nvt_thp_boot_firmware_flags(struct regmap *map,
				       const struct firmware *firmware,
				       const struct nvt_thp_firmware *image,
				       u16 *payload_length, bool probe_cmds);

int nvt_thp_boot_firmware(struct regmap *map, const struct firmware *firmware,
			  const struct nvt_thp_firmware *image, u16 *payload_length)
{
	return nvt_thp_boot_firmware_flags(map, firmware, image, payload_length,
					   true);
}

int nvt_thp_boot_firmware_reinit(struct regmap *map,
				 const struct firmware *firmware,
				 const struct nvt_thp_firmware *image,
				 u16 *payload_length)
{
	return nvt_thp_boot_firmware_flags(map, firmware, image, payload_length,
					   false);
}

static int nvt_thp_boot_firmware_flags(struct regmap *map,
				       const struct firmware *firmware,
				       const struct nvt_thp_firmware *image,
				       u16 *payload_length, bool probe_cmds)
{
	u8 identity[6], zeros[6] = {};
	unsigned int value, index;
	int ret;

	/*
	 * Exact vendor probe-time host command sequence (captured via kprobe
	 * on spi_sync in the Android vendor driver).  Only at the initial probe:
	 * on a re-init after a panel power cycle these commands time out (100 x
	 * ~1.25 ms each) and only delay the recovery by ~750 ms.
	 */
	if (probe_cmds) {
		struct device *dev = regmap_get_device(map);
		int a, b, c, d, e, f;

		a = nvt_host_cmd2(map, 0x7b, 0x04, 0x01);
		b = nvt_host_cmd(map, 0x76, 0x01);
		c = nvt_host_cmd(map, 0x79, 0x02);
		d = nvt_host_cmd1(map, 0xba);
		e = nvt_host_cmd(map, 0x79, 0x00);
		f = nvt_host_cmd(map, 0x75, 0x00);
		dev_dbg(dev, "touch probe cmds 7b/76/79/ba/79/75: %d %d %d %d %d %d\n",
			 a, b, c, d, e, f);
	}
	/* Vendor trace shows no 0x7fff80 write at all. */
	usleep_range(1000, 2000);
	ret = nvt_boot_reset(map);
	if (ret)
		return ret;
	ret = regmap_bulk_write(map, 0x1fb104, zeros, sizeof(zeros));
	if (ret)
		return ret;
	ret = regmap_bulk_read(map, 0x1fb104, identity, sizeof(identity));
	if (ret)
		return ret;
	if (identity[4] != 0x65 || identity[5] != 0x03)
		return -ENODEV;
	ret = regmap_read(map, 0x1fb12c, &value);
	if (ret)
		return ret;
	if (value & BIT(0))
		return -EOPNOTSUPP;
	ret = nvt_boot_reset(map);
	if (ret)
		return ret;
	ret = nvt_crc_bank(map, &image->partitions[0], 0x1fcc30, 0x1fcc34, 0x1fcc38);
	if (ret)
		return ret;
	ret = nvt_crc_bank(map, &image->partitions[1], 0x1fcc40, 0x1fcc44, 0x1fcc48);
	if (ret)
		return ret;
	ret = regmap_write(map, NVT_AUTO_COPY, 0x56);
	if (ret)
		return ret;
	for (index = 0; index < image->count; ++index) {
		const struct nvt_thp_partition *part = &image->partitions[index];

		if (!part->length)
			continue;
		{
			u32 done;

			for (done = 0; done < part->length; done += NVT_WRITE_CHUNK) {
				u32 n = min_t(u32, NVT_WRITE_CHUNK, part->length - done);

				ret = regmap_bulk_write(map, part->address + done,
							firmware->data + part->offset + done, n);
				if (ret)
					return ret;
			}
		}
	}
	/* Vendor nvt_wait_auto_copy(). */
	ret = regmap_read_poll_timeout(map, NVT_AUTO_COPY, value, !value, 1000, 200000);
	if (ret)
		return ret;
	/* Vendor writes 0xff to the auto-copy register after the download. */
	ret = regmap_write(map, NVT_AUTO_COPY, 0xff);
	if (ret)
		return ret;
	/* Vendor clears the report buffer, then sends host command 0xae. */
	{
		u8 z[7] = {};
		int hc;

		regmap_bulk_write(map, 0x130900, z, sizeof(z));
		hc = nvt_host_cmd1(map, 0xae);
		dev_dbg(regmap_get_device(map), "touch init 0xae:%d\n", hc);
	}
	/* Vendor nvt_boot_ready(): 0x1fb50d = 1, then ~5 ms. */
	ret = regmap_write(map, NVT_BOOT_READY, 1);
	if (ret)
		return ret;
	usleep_range(5000, 6000);
	/* Vendor nvt_check_fw_reset_state(0xa0): 6 bytes at client base,
	 * status is the second byte, which is value[0] for this transport. */
	{
		u8 st[6];
		u8 fw[39];
		int i;

		for (i = 0; i < 100; i++) {
			if (regmap_bulk_read(map, NVT_FW_STATE, st, sizeof(st)))
				return -EIO;
			if (st[0] >= 0xa0 && st[0] <= 0xaf)
				break;
			usleep_range(10000, 11000);
		}
		if (i >= 100)
			return -ETIMEDOUT;
			/* Vendor post-boot commands (captured): 0x7b 00, 0x51, 0x53, then 0xbc. */
	{
		struct device *dev = regmap_get_device(map);
		int a = nvt_host_cmd(map, 0x7b, 0x00);
		int b = nvt_host_cmd1(map, 0x51);
		int c = nvt_host_cmd1(map, 0x53);
		int d = nvt_host_cmd1(map, 0xbc);

		dev_dbg(dev, "touch post-boot 7b/51/53/bc: %d %d %d %d\n",
			 a, b, c, d);
	}
	/* Vendor nvt_get_fw_info(). */
		if (!regmap_bulk_read(map, NVT_FWINFO, fw, sizeof(fw)))
		{
			u8 rep[0x6d];
			int j;
			u8 sum = 0;

			if (!regmap_bulk_read(map, 0x130900, rep, sizeof(rep))) {
				for (j = 1; j < 0x40; j++)
					sum += rep[j];
			}
		}

	}
	*payload_length = 300;
	return 0;
}

/*
 * Downstream's resume/restore command sequence, captured with a kprobe on
 * nvt_cmd_store() in the vendor module while the panel was coming back up:
 *
 *   0xBA
 *   0x7F 0x01 0x00 00 00 00 00 00
 *   0x79 0x00
 *   0x7B 0x00
 *   0x7C 0x00
 *   0x7B 0x04 0x01      (pen type)
 *   0x76 0x00
 *   0x75 0x00
 *   0x7F 0x02 0x00
 *
 * The second 0x7B command is nvt_support_pen_set(): a single host command byte
 * with a 0..3 value, not the piano stack's 0xBF/0x04 "stylus enable" extended
 * command.  Stock sends 0 there (update_pen_status() with no pen consumer), and
 * Lenovo's Android HAL - vendor.lenovo.hardware.touchscreen-service, AIDL
 * setPenBle() - can later write /proc/support_pen, i.e. the same command with
 * 1, when the pen stack connects.  The kernel side of that path is
 * nvt_support_pen_store() -> nvt_cmd_store({0x7b, value}).
 *
 * Stock Android runs the whole pen pipeline with the value left at 0 (verified
 * over adb: /proc/support_pen = 0, /proc/pen_type = 0 while the pen draws with
 * full 12-bit pressure), so 0 is the default here too and the pen packet comes
 * from the firmware as long as the panel is not driven above 120 Hz.
 */
static int pen_support;
module_param(pen_support, int, 0644);
MODULE_PARM_DESC(pen_support,
		 "Host command 0x7b (pen reporting): 0 = stock default, 1 = force it on");

/*
 * Bring-up knob: raw host command bytes appended after the restore sequence,
 * e.g. extra_cmd="c1 04" for the vendor's nvt_set_pen_normal_mode()
 * (0xC1 0x04 written to EVENT_MAP_HOST_CMD), or "c1 02 00 01" for
 * nvt_set_pen_inband_mode_1(0, 1).  Empty by default.
 */
static char *extra_cmd;
module_param(extra_cmd, charp, 0400);
MODULE_PARM_DESC(extra_cmd,
		 "Extra host command bytes (hex, space separated) sent after the restore sequence");

static int nvt_host_write(struct regmap *map, const u8 *buf, size_t len)
{
	u8 st[2] = {};
	int i, ret;

	ret = regmap_bulk_write(map, NVT_HOST_CMD, buf, len);
	if (ret)
		return ret;
	for (i = 0; i < 100; i++) {
		usleep_range(1000, 1500);
		ret = regmap_bulk_read(map, NVT_HOST_CMD, st, sizeof(st));
		if (ret)
			return ret;
		if (st[0] == 0)
			return 0;
	}
	return -ETIMEDOUT;
}

/* Optional raw host command from the extra_cmd module parameter. */
static int nvt_thp_extra_command(struct regmap *map,
				 struct nvt_thp_command_status *last)
{
	u8 buf[8];
	size_t len = 0;
	const char *p = extra_cmd;
	int ret;

	if (!p || !*p)
		return 0;
	while (*p && len < sizeof(buf)) {
		unsigned int value;
		char *end;

		while (*p == ' ' || *p == ',')
			p++;
		if (!*p)
			break;
		value = simple_strtoul(p, &end, 16);
		if (end == p || value > 0xff)
			return -EINVAL;
		buf[len++] = value;
		p = end;
	}
	if (!len)
		return 0;
	ret = nvt_host_write(map, buf, len);
	last->sub_command = buf[0];
	last->status = ret;
	return ret;
}

int nvt_thp_apply_host_commands(struct regmap *map, struct nvt_thp_command_status *last)
{
	static const struct {
		u8 data[8];
		u8 len;
	} restore[] = {
		{ { 0xBA }, 1 },
		/* Values below match the stock Android runtime (see the comment). */
		{ { 0x7F, 0x00, 0x00 }, 8 },	/* edge grid zone 0,0,0,0 */
		{ { 0x79, 0x00 }, 2 },
		{ { 0x7B, 0x00 }, 2 },		/* pen support (pen_support) */
		{ { 0x7C, 0x00 }, 2 },
		{ { 0x7B, 0x04, 0x00 }, 3 },	/* pen type 0, as stock runs it */
		{ { 0x76, 0x00 }, 2 },
		{ { 0x75, 0x00 }, 2 },
		{ { 0x7F, 0x02, 0x00 }, 3 },
	};
	u8 command[8];
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(restore); i++) {
		memcpy(command, restore[i].data, restore[i].len);
		/* Pick the pen-reporting value instead of stock's hard-coded 0. */
		if (command[0] == 0x7b && restore[i].len == 2)
			command[1] = pen_support & 1;
		ret = nvt_host_write(map, command, restore[i].len);
		last->sub_command = command[0];
		last->status = ret;
		if (ret)
			return ret;
	}

	return nvt_thp_extra_command(map, last);
}
