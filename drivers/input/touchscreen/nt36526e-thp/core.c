// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * SPI paging/host-download protocol from the piano vendor driver.
 * NTP1 framing follows ianchb's NT36526E THP stream. See README.rst.
 */
#include <drm/drm_panel.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/kref.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/uaccess.h>
#include "firmware.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36526E_THP_KUNIT_TEST)
#include <kunit/test.h>
#endif

#define NVT_WRITE_SIZE	(63 * 1024)
/* 1 = reconfigure the controller after the panel powers up (vendor behaviour) */
static bool reinit_on_wake = true;
module_param(reinit_on_wake, bool, 0644);
MODULE_PARM_DESC(reinit_on_wake, "Reconfigure the touch controller after a panel power cycle");

static int rot = 3;
static int mt = 1;
module_param(mt, int, 0644);
/*
 * The keep-awake command shares the report channel with touch data and
 * there is no lock between this work and the IRQ handler, so a command
 * response could be parsed as a permanently pressed finger (phantom
 * touch, IRQ storm, dead touchscreen).  Disabled by default; set it from
 * the module parameter only if the controller turns out to need it.
 */
static uint keepawake_ms;
static int dbg;
/* dump the first N raw reports (pen protocol discovery) */
static int dump_frames;
module_param(dump_frames, int, 0644);
module_param(dbg, int, 0644);
module_param(keepawake_ms, uint, 0644);
MODULE_PARM_DESC(mt, "Report as a real MT touchscreen instead of an absolute pointer");
module_param(rot, int, 0644);
MODULE_PARM_DESC(rot, "Axis transform: 0=transpose 1=cw 2=transpose+180 3=ccw");
#define NVT_READ_SIZE	8192
#define NVT_FRAME_PREFIX	257
#define NVT_STREAM_MAGIC	0x3150544e
#define NVT_FRAME_VALID	BIT(0)
#define NVT_FRAME_EPOCH	BIT(1)
#define NVT_IRQ_ENABLED	0
/* Vendor nvt_check_fw_reset_state() budget for post-INIT states: 50 x 10 ms. */
#define NVT_FIRST_FRAME_MS	500

struct nvt_stream_header {
	__le32 magic;
	__le16 header_length;
	__le16 frame_length;
	__le64 sequence;
	__le64 timestamp_ns;
	__le16 checksum;
	__le16 flags;
	__le32 firmware_length_words;
} __packed;

struct nvt_thp {
	struct spi_device *spi;
	struct regmap *map;
	struct input_dev *input;
	struct input_dev *pen_input;
	bool pen_down;
	struct miscdevice misc;
	struct drm_panel_follower follower;
	struct kref ref;
	struct mutex lifecycle_lock; /* Serializes open, close, removal and PM. */
	struct mutex read_lock; /* Keeps the FIFO single-consumer across shared fds. */
	wait_queue_head_t wait;
	struct kfifo fifo;
	const char *firmware_name;
	u8 *tx, *rx, *record;
	u8 *rtx, *rrx;
	size_t max_read;
	u16 frame_length;
	u64 sequence;
	unsigned long flags;
	int error;
	bool dead, opened, running, epoch, suspended, panel_off;
	int empty_run;
	u16 last_x, last_y;
	u8 last_pressure;
	bool pressed;
	struct delayed_work wd;
	struct delayed_work ka;
	int diag_n;
};

static_assert(sizeof(struct nvt_stream_header) == 32);

static void nvt_destroy(struct kref *ref)
{
	struct nvt_thp *nvt = container_of(ref, struct nvt_thp, ref);

	kfifo_free(&nvt->fifo);
	kfree(nvt->record);
	kfree(nvt->rx);
	kfree(nvt->tx);
	kfree(nvt->misc.name);
	kfree(nvt);
}

static void nvt_put(void *data)
{
	struct nvt_thp *nvt = data;

	kref_put(&nvt->ref, nvt_destroy);
}

static int nvt_transfer(struct nvt_thp *nvt, size_t size, bool read)
{
	struct spi_transfer transfer = {
		.tx_buf = nvt->tx,
		.rx_buf = read ? nvt->rx : NULL,
		.len = size,
	};
	struct spi_message message;

	spi_message_init_with_transfers(&message, &transfer, 1);
	/*
	 * Only the spi_sync() status is meaningful: spi-geni-qcom's GPI DMA
	 * callback sets cur_msg->status before finalizing the transfer, so the
	 * SPI core never adds the transfer to actual_length on success.
	 */
	return spi_sync(nvt->spi, &message);
}

/*
 * Vendor equivalent of CTP_SPI_READ(buf, 0x6d): one transfer with 0x00 as the
 * address byte.  The chip is already paged to the event map, so skipping the
 * separate page-select should shave a transfer off every report.
 */
static int nvt_read_report(struct nvt_thp *nvt, u8 *buf, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = nvt->rtx,
		.rx_buf = nvt->rrx,
		.len = len + 2,
	};
	struct spi_message msg;

	memset(nvt->rtx, 0, len + 2);
	nvt->rtx[0] = 0x00;
	spi_message_init_with_transfers(&msg, &xfer, 1);
	if (spi_sync(nvt->spi, &msg))
		return -EIO;
	memcpy(buf, nvt->rrx + 2, len);
	return 0;
}

static int nvt_select_page(struct nvt_thp *nvt, u32 address)
{
	nvt->tx[0] = 0xff;
	nvt->tx[1] = address >> 15;
	nvt->tx[2] = address >> 7;
	return nvt_transfer(nvt, 3, false);
}

static int nvt_spi_read(void *context, const void *reg, size_t reg_size,
			void *value, size_t size)
{
	struct nvt_thp *nvt = context;
	u32 address;
	int ret;

	if (reg_size != sizeof(u32) || !size || size > nvt->max_read)
		return -EINVAL;
	address = get_unaligned((const u32 *)reg);
	ret = nvt_select_page(nvt, address);
	if (ret)
		return ret;
	memset(nvt->tx, 0, size + 2);
	nvt->tx[0] = address & 0x7f;
	ret = nvt_transfer(nvt, size + 2, true);
	if (!ret)
		memcpy(value, nvt->rx + 2, size);
	return ret;
}

static int nvt_spi_write(void *context, const void *data, size_t count)
{
	struct nvt_thp *nvt = context;
	u32 address;
	int ret;

	if (count <= sizeof(u32) || count - sizeof(u32) > NVT_WRITE_SIZE)
		return -EINVAL;
	address = get_unaligned((const u32 *)data);
	if (dbg)
		dev_dbg(&nvt->spi->dev, "W %06x %*ph\n", address,
			 (int)(count - 4), (const u8 *)data + 4);
	ret = nvt_select_page(nvt, address);
	if (ret)
		return ret;
	nvt->tx[0] = (address & 0x7f) | 0x80;
	memcpy(nvt->tx + 1, (const u8 *)data + sizeof(u32), count - sizeof(u32));
	return nvt_transfer(nvt, count - sizeof(u32) + 1, false);
}

static bool nvt_precious_register(struct device *dev, unsigned int reg)
{
	/* Do not turn a debugfs register dump into controller commands. */
	return true;
}

static bool nvt_valid_frame(const u8 *payload, size_t size)
{
	u32 words = get_unaligned_le32(payload + 8), sum;
	u16 crc = get_unaligned_le16(payload + 4);
	size_t offset;

	if (payload[0] != 0xa5 || words > (size - 20) / 4 ||
	    get_unaligned_le32(payload + 16) != ~words ||
	    get_unaligned_le16(payload + 12) != (u16)~crc)
		return false;
	sum = crc;
	for (offset = 20; offset < 20 + (size_t)words * 4; offset += 2)
		sum += get_unaligned_le16(payload + offset);
	return !(sum & 0xffff);
}

/*
 * Pen report, reverse engineered from nvt_ts_work_func() in the vendor module
 * (stock-fw/nvt_36xxx.ko, disassembly at nvt_ts_work_func+0x914..0xd6c).
 *
 * The vendor calls CTP_SPI_READ(buf, 0x6d), which keeps the SPI address byte in
 * buf[0] and stores the first report byte in buf[1] (second dummy byte is
 * dropped), so its pen offsets are one higher than ours: this transport copies
 * rx+2 (i.e. vendor buf[1]) into rep[0].  Vendor byte n is therefore rep[n-1].
 * All 16-bit fields are big endian.  In rep[] that gives:
 *   [65]     pen event: 0xff = out of range, 1 = pen, 2 = eraser,
 *            0xf0 = pen MAC packet, 0xf1 = pen identity packet
 *   [66..67] X, [68..69] Y, [70..71] pressure (12 bit),
 *   [72] tilt X, [73] tilt Y, [74..75] hover distance,
 *   [76] bit0 BTN_STYLUS / bit1 BTN_STYLUS2, bits 2+ touch info,
 *   [77] pen battery.
 * Vendor buf[65] (our rep[64]) is the report checksum over buf[1..64].
 */
static void nvt_report_pen(struct nvt_thp *nvt, const u8 *rep)
{
	struct input_dev *pen = nvt->pen_input;
	u8 type = rep[65];
	u16 x, y, pressure, distance, sx, sy;
	s8 tilt_x, tilt_y;
	bool was_down;

	if (!pen)
		return;

	/*
	 * Vendor also receives identity packets on the same byte: 0xf0 carries the
	 * pen MAC address, 0xf1 the pen identity (response to host command 0x7a).
	 * They prove the pen link is alive even when no pen data flows.
	 */
	if (type == 0xf0 || type == 0xf1) {
		dev_dbg_ratelimited(&nvt->spi->dev, "pen id packet type %u %*ph\n",
				     type, 13, rep + 65);
		return;
	}

	if (type != 1 && type != 2) {
		if (!READ_ONCE(nvt->pen_down))
			return;
		WRITE_ONCE(nvt->pen_down, false);
		input_report_key(pen, BTN_TOOL_PEN, 0);
		input_report_key(pen, BTN_TOOL_RUBBER, 0);
		input_report_key(pen, BTN_TOUCH, 0);
		input_report_key(pen, BTN_STYLUS, 0);
		input_report_key(pen, BTN_STYLUS2, 0);
		input_report_abs(pen, ABS_PRESSURE, 0);
		input_report_abs(pen, ABS_DISTANCE, 0);
		input_sync(pen);
		return;
	}

	x = (rep[66] << 8) | rep[67];
	y = (rep[68] << 8) | rep[69];
	pressure = (rep[70] << 8) | rep[71];
	tilt_x = (s8)rep[72];
	tilt_y = (s8)rep[73];
	/* The controller reports tilt past the vendor's +-60 degree axis range. */
	tilt_x = clamp_t(int, tilt_x, -60, 60);
	tilt_y = clamp_t(int, tilt_y, -60, 60);
	distance = (rep[74] << 8) | rep[75];
	if (x >= 21901 || y >= 35041)
		return;

	switch (rot) {
	case 0:  sx = y;             sy = x;             break;
	case 2:  sx = 35040 - y;     sy = 21900 - x;     break;
	case 3:  sx = y;             sy = 21900 - x;     break;
	default: sx = 35040 - y;     sy = x;             break;
	}

	was_down = READ_ONCE(nvt->pen_down);
	WRITE_ONCE(nvt->pen_down, true);
	/* One line per contact, enough to confirm the pen packet on hardware. */
	if (!was_down)
		dev_dbg(&nvt->spi->dev, "pen packet type %u %*ph\n", type,
			 16, rep + 64);
	input_report_abs(pen, ABS_X, sx);
	input_report_abs(pen, ABS_Y, sy);
	input_report_abs(pen, ABS_PRESSURE, pressure);
	input_report_abs(pen, ABS_TILT_X, tilt_x);
	input_report_abs(pen, ABS_TILT_Y, tilt_y);
	input_report_abs(pen, ABS_DISTANCE, distance ? 1 : 0);
	/* Vendor only (re)asserts the tool while the pen is in range. */
	if (pressure || distance) {
		input_report_key(pen, BTN_TOOL_PEN, type == 1);
		input_report_key(pen, BTN_TOOL_RUBBER, type == 2);
	}
	input_report_key(pen, BTN_TOUCH, pressure > 0);
	input_report_key(pen, BTN_STYLUS, rep[76] & 1);
	input_report_key(pen, BTN_STYLUS2, (rep[76] >> 1) & 1);
	input_sync(pen);
}

static void nvt_report_touch(struct nvt_thp *nvt, const u8 *rep)
{
	struct input_dev *input = nvt->input;
	bool was __attribute__((unused)) = nvt->pressed;
	int i, slots = 0;

	for (i = 0; i < 10; i++) {
		const u8 *s = rep + i * 6;
		u16 x, y, sx, sy;

		if (s[0] == 0xff)
			continue;
		x = (s[1] << 8) | s[2];
		y = (s[3] << 8) | s[4];
		if (x >= 21901 || y >= 35041)
			continue;
		switch (rot) {
		case 0:  sx = y;             sy = x;             break;
		case 2:  sx = 35040 - y;     sy = 21900 - x;     break;
		case 3:  sx = y;             sy = 21900 - x;     break;
		default: sx = 35040 - y;     sy = x;             break;
		}
		if (mt) {
			input_mt_slot(input, slots);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
			input_report_abs(input, ABS_MT_POSITION_X, sx);
			input_report_abs(input, ABS_MT_POSITION_Y, sy);
			input_report_abs(input, ABS_MT_PRESSURE, s[5]);
		} else {
			input_report_abs(input, ABS_X, sx);
			input_report_abs(input, ABS_Y, sy);
			input_report_abs(input, ABS_PRESSURE, s[5]);
		}
		nvt->last_x = sx;
		nvt->last_y = sy;
		nvt->last_pressure = s[5];
		slots++;
	}

	/* The chip sends exactly one all-0xff report on release: react at once. */
	nvt->pressed = slots > 0;
	if (mt) {
		input_mt_drop_unused(input);
		input_mt_sync_frame(input);
	}
	input_report_key(input, BTN_TOUCH, slots > 0);
	if (!mt)
		input_report_key(input, BTN_LEFT, slots > 0);
	input_sync(input);
}

static irqreturn_t nvt_irq(int irq, void *data)
{
	struct nvt_thp *nvt = data;
	u8 rep[0x6d];
	int ret;

	/* The IRQ is enabled as soon as the firmware boots, but the input
	 * device is registered later: never touch it before it exists. */
	if (!READ_ONCE(nvt->running) || !READ_ONCE(nvt->input))
		return IRQ_HANDLED;
	/* Standard Novatek report: 0x6d bytes at client_base, 6 bytes per slot. */
	{
		static int nread;
		ktime_t t0 = ktime_get();

		ret = nvt_read_report(nvt, rep, sizeof(rep));
		if (ret) {
			dev_err_ratelimited(&nvt->spi->dev, "SPI report read failed: %d\n", ret);
			return IRQ_HANDLED;
		}
		if (nread < 10 && rep[0] != 0xff) {
			nread++;
			dev_dbg(&nvt->spi->dev, "read %lldus rep %*ph\n",
				 ktime_us_delta(ktime_get(), t0), 6, rep);
		}
		if (dump_frames > 0) {
			dump_frames--;
			/* 80 bytes reach through the checksum into the pen block. */
			dev_dbg(&nvt->spi->dev, "frame %*ph\n", 80, rep);
		}
	}
	nvt_report_touch(nvt, rep);
	nvt_report_pen(nvt, rep);
	return IRQ_HANDLED;
}

static void nvt_stop_irq(struct nvt_thp *nvt)
{
	/* Keep the IRQ armed: the input device relies on it without a reader. */
}

/* Caller holds lifecycle_lock, with the IRQ stopped. */
static int nvt_start_stream(struct nvt_thp *nvt)
{
	const struct firmware *firmware;
	struct nvt_thp_command_status last;
	struct nvt_thp_firmware image;
	s64 first_frame_ms;
	u16 payload_length;
	int ret;

	lockdep_assert_held(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->error, 0);
	if (READ_ONCE(nvt->running)) {
		/* Firmware already booted at probe for the input device. */
		if (test_and_set_bit(NVT_IRQ_ENABLED, &nvt->flags) == 0)
			enable_irq(nvt->spi->irq);
		return 0;
	}
	ret = request_firmware(&firmware, nvt->firmware_name, &nvt->spi->dev);
	if (ret)
		return ret;
	ret = nvt_thp_parse_firmware(firmware->data, firmware->size, &image);
	if (!ret) {
		ret = nvt_thp_boot_firmware_reinit(nvt->map, firmware, &image,
						   &payload_length);
		nvt_thp_free_firmware(&image);
	}
	release_firmware(firmware);
	if (ret)
		return ret;
	/* IRQ packets must fit one transaction; do not split a changing frame. */
	if (payload_length < 64 || payload_length > nvt->max_read - (NVT_FRAME_PREFIX - 1)) {
		ret = -EMSGSIZE;
		return ret;
	}
	nvt->frame_length = NVT_FRAME_PREFIX + payload_length;
	WRITE_ONCE(nvt->epoch, true);
	WRITE_ONCE(nvt->running, true);
	if (test_and_set_bit(NVT_IRQ_ENABLED, &nvt->flags) == 0)
		enable_irq(nvt->spi->irq);
	/*
	 * No first-frame wait: this panel runs the standard Novatek report
	 * format, so the THP handshake never completes (the IRQ path does not
	 * clear nvt->epoch) and waiting for it only added a fixed 500 ms to
	 * every re-init, which is what made the touch come back late after a
	 * blank.  Downstream sends the host commands straight after the boot as
	 * well.
	 */
	first_frame_ms = 0;
	ret = nvt_thp_apply_host_commands(nvt->map, &last);
	if (ret) {
		dev_warn(&nvt->spi->dev,
			 "THP host command BF %02x failed: %d (ignored)\n",
			 last.sub_command, ret);
		ret = 0;
	}
	ret = READ_ONCE(nvt->error);
	if (ret)
		goto stop;
	dev_dbg(&nvt->spi->dev,
		 "THP first frame after %lld ms; vendor restore command sequence replayed\n",
		 first_frame_ms);
	dev_dbg(&nvt->spi->dev, "THP stream ready, payload %u bytes\n", payload_length);
	return 0;
stop:
	nvt_stop_irq(nvt);
	return ret;
}

static int nvt_panel_disabling(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);

	/*
	 * Called from the DRM commit thread: never take lifecycle_lock here, the
	 * watchdog may hold it for the whole firmware upload (~150 ms) and the
	 * commit would stall, which is visible as a display init hiccup.
	 */
	if (!READ_ONCE(nvt->panel_off)) {
		nvt_stop_irq(nvt);
		WRITE_ONCE(nvt->panel_off, true);
	}
	return 0;
}

static int nvt_panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);

	/*
	 * The panel has just been powered up and re-initialised, and the reset
	 * line it shares with the touch controller released.  The controller
	 * therefore has to be configured again; mark it here so the re-init in
	 * nvt_watchdog cannot race with the panel bring-up.
	 */
	if (reinit_on_wake) {
		WRITE_ONCE(nvt->running, false);
		/* The panel is coming up: drop the "panel off" guard. */
		WRITE_ONCE(nvt->panel_off, false);
		/*
		 * Start reconfiguring now instead of after drm_panel_enable():
		 * the firmware upload (~150 ms) then overlaps the display
		 * bring-up, and nvt_watchdog retries every 100 ms until the
		 * controller accepts it.  Only queues work, never blocks the
		 * commit.
		 */
		mod_delayed_work(system_wq, &nvt->wd, 0);
	}
	return 0;
}

static int nvt_panel_enabled(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);

	/*
	 * This runs in the DRM commit thread.  Never take lifecycle_lock and
	 * never download firmware here: the upload takes ~150 ms and would stall
	 * the modeset (visible as a display init hiccup).  nvt_panel_prepared()
	 * already queued the reconfiguration, nvt_watchdog() performs it in a
	 * workqueue.
	 */
	if (READ_ONCE(nvt->dead))
		return 0;
	WRITE_ONCE(nvt->error, 0);
	WRITE_ONCE(nvt->panel_off, false);
	/* Preserve queued bytes, including any partially read NTP1 record. */
	wake_up_interruptible(&nvt->wait);
	return 0;
}

static int nvt_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);


	/* Commit thread context: atomics only, see nvt_panel_disabling(). */
	if (!READ_ONCE(nvt->panel_off)) {
		nvt_stop_irq(nvt);
		WRITE_ONCE(nvt->panel_off, true);
	}
	if (reinit_on_wake)
		WRITE_ONCE(nvt->running, false);
	return 0;
}

static const struct drm_panel_follower_funcs nvt_panel_follower_funcs = {
	/* TDDI scanning needs the display timing, not just panel power. */
	.panel_prepared = nvt_panel_prepared,
	.panel_enabled = nvt_panel_enabled,
	.panel_disabling = nvt_panel_disabling,
	.panel_unpreparing = nvt_panel_unpreparing,
};

static int nvt_open(struct inode *inode, struct file *file)
{
	struct nvt_thp *nvt = container_of(file->private_data, struct nvt_thp, misc);
	int ret;

	if (file->f_mode & FMODE_WRITE)
		return -EPERM;
	ret = mutex_lock_interruptible(&nvt->lifecycle_lock);
	if (ret)
		return ret;
	if (nvt->dead) {
		ret = -ENODEV;
		goto unlock;
	}
	if (nvt->opened || nvt->suspended) {
		ret = -EBUSY;
		goto unlock;
	}
	kfifo_reset(&nvt->fifo);
	WRITE_ONCE(nvt->error, 0);
	/* udev may open the stream before the panel has been enabled. */
	if (!nvt->panel_off) {
		ret = nvt_start_stream(nvt);
		if (ret) {
			dev_err(&nvt->spi->dev, "THP stream start failed: %d\n", ret);
			goto unlock;
		}
	}
	nvt->opened = true;
	kref_get(&nvt->ref);
	file->private_data = nvt;
	stream_open(inode, file);
unlock:
	mutex_unlock(&nvt->lifecycle_lock);
	return ret;
}

static int nvt_release(struct inode *inode, struct file *file)
{
	struct nvt_thp *nvt = file->private_data;

	mutex_lock(&nvt->lifecycle_lock);
	if (!nvt->dead)
		nvt_stop_irq(nvt);
	nvt->opened = false;
	mutex_unlock(&nvt->lifecycle_lock);
	nvt_put(nvt);
	return 0;
}

static bool nvt_read_ready(struct nvt_thp *nvt)
{
	return READ_ONCE(nvt->dead) ||
	       (!READ_ONCE(nvt->panel_off) &&
		(READ_ONCE(nvt->error) || !kfifo_is_empty(&nvt->fifo)));
}

static ssize_t nvt_read(struct file *file, char __user *buffer, size_t count, loff_t *position)
{
	struct nvt_thp *nvt = file->private_data;
	unsigned int copied;
	int ret;

	if (!count)
		return 0;
	ret = mutex_lock_interruptible(&nvt->read_lock);
	if (ret)
		return ret;
	if (!nvt_read_ready(nvt)) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		ret = wait_event_interruptible(nvt->wait, nvt_read_ready(nvt));
		if (ret)
			goto unlock;
	}
	if (READ_ONCE(nvt->dead)) {
		ret = -ENODEV;
		goto unlock;
	}
	ret = READ_ONCE(nvt->error);
	if (!ret) {
		ret = kfifo_to_user(&nvt->fifo, buffer, min_t(size_t, count, UINT_MAX), &copied);
		if (!ret)
			ret = copied;
	}
unlock:
	mutex_unlock(&nvt->read_lock);
	return ret;
}

static __poll_t nvt_poll(struct file *file, poll_table *wait)
{
	struct nvt_thp *nvt = file->private_data;

	poll_wait(file, &nvt->wait, wait);
	if (READ_ONCE(nvt->dead))
		return EPOLLERR | EPOLLHUP;
	if (READ_ONCE(nvt->panel_off))
		return 0;
	if (READ_ONCE(nvt->error))
		return EPOLLERR | EPOLLHUP;
	return kfifo_is_empty(&nvt->fifo) ? 0 : EPOLLIN | EPOLLRDNORM;
}

static const struct file_operations nvt_fops = {
	.owner = THIS_MODULE,
	.open = nvt_open,
	.release = nvt_release,
	.read = nvt_read,
	.poll = nvt_poll,
};

/*
 * The panel and the touch sensor share one TDDI die: a display mode change or
 * a panel reset also resets the touch firmware.  Re-download it when the
 * boot state stops reporting 0xa0..0xaf.
 */
/* Vendor repeats host commands 0x51/0x53 after boot. */
static int nvt_cmd51_53(struct nvt_thp *nvt)
{
	u8 c, st[2];
	int i, ok = 0;

	for (c = 0; c < 2; c++) {
		u8 cmd = c ? 0x53 : 0x51;

		if (regmap_bulk_write(nvt->map, 0x130950, &cmd, 1))
			continue;
		for (i = 0; i < 20; i++) {
			usleep_range(500, 800);
			if (!regmap_bulk_read(nvt->map, 0x130950, st, 2) && st[0] == 0) {
				ok++;
				break;
			}
		}
	}
	return ok;
}

static void nvt_keepawake_work(struct work_struct *work)
{
	struct nvt_thp *nvt = container_of(work, struct nvt_thp, ka.work);

	if (READ_ONCE(nvt->running) && keepawake_ms)
		nvt_cmd51_53(nvt);
	schedule_delayed_work(&nvt->ka, msecs_to_jiffies(keepawake_ms));
}

static void nvt_watchdog(struct work_struct *work)
{
	struct nvt_thp *nvt = container_of(work, struct nvt_thp, wd.work);
	u8 st[6];
	int ret;

	if (READ_ONCE(nvt->dead))
		return;

	if (READ_ONCE(nvt->panel_off)) {
		/*
		 * The panel is off, so the controller is off with it: there is
		 * nothing to repair until panel_enabled() runs again.  Do not
		 * poke the bus while the display is powering down/up.
		 */
		schedule_delayed_work(&nvt->wd, msecs_to_jiffies(3000));
		return;
	}

	if (READ_ONCE(nvt->running)) {
		if (!regmap_bulk_read(nvt->map, 0x130960, st, sizeof(st)) &&
		    st[0] >= 0xa0 && st[0] <= 0xaf) {
			schedule_delayed_work(&nvt->wd, msecs_to_jiffies(3000));
			return;
		}
		dev_dbg(&nvt->spi->dev, "touch fw lost (state %02x), re-downloading\n",
			 st[0]);
		WRITE_ONCE(nvt->running, false);
	}

	/*
	 * Reconfigure the controller exactly like the probe path: firmware
	 * download, command replay, and wait for the first valid frame.  The
	 * first-frame wait matters because the TDDI part only starts reporting
	 * once the display timing is running; a boot that "succeeds" without it
	 * leaves the controller silent.
	 */
	mutex_lock(&nvt->lifecycle_lock);
	ret = nvt_start_stream(nvt);
	mutex_unlock(&nvt->lifecycle_lock);

	if (ret)
		dev_warn(&nvt->spi->dev, "touch re-init failed: %d, retrying\n", ret);

	/*
	 * The download can "succeed" on a controller that does not scan (it only
	 * reports once the display timing is running).  Clear running again so
	 * the next 100 ms tick retries instead of waiting for the 3 s watchdog.
	 */
	if (!ret) {
		u8 st[6];

		if (regmap_bulk_read(nvt->map, 0x130960, st, sizeof(st)) ||
		    st[0] < 0xa0 || st[0] > 0xaf) {
			dev_warn(&nvt->spi->dev,
				 "touch state %02x after re-init, retrying\n", st[0]);
			WRITE_ONCE(nvt->running, false);
		}
	}

	/*
	 * Retry quickly until the controller is back: the first attempt after a
	 * panel power cycle can arrive before the display timing is stable.
	 */
	schedule_delayed_work(&nvt->wd,
			      msecs_to_jiffies(READ_ONCE(nvt->running) ? 3000 : 100));
}

static int nvt_probe(struct spi_device *spi)
{
	struct regmap_config config = {
		/* Config callbacks use native-endian register formatting. */
		.reg_bits = 32,
		.val_bits = 8,
		.read = nvt_spi_read,
		.write = nvt_spi_write,
		.max_register = 0x7fffff,
		.cache_type = REGCACHE_NONE,
		.precious_reg = nvt_precious_register,
	};
	struct nvt_thp *nvt;
	size_t transfer_max;
	int ret;

	if (spi->irq <= 0 || (spi->mode & (SPI_CPOL | SPI_CPHA)) ||
	    (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX))
		return -EINVAL;
	transfer_max = min(spi_max_transfer_size(spi), spi_max_message_size(spi));
	if (transfer_max < NVT_FRAME_PREFIX + 64 + 1)
		return -EMSGSIZE;
	nvt = kzalloc_obj(*nvt);
	if (!nvt)
		return -ENOMEM;
	kref_init(&nvt->ref);
	ret = devm_add_action_or_reset(&spi->dev, nvt_put, nvt);
	if (ret)
		return ret;
	nvt->spi = spi;
	nvt->max_read = min_t(size_t, NVT_READ_SIZE - 2, transfer_max - 2);
	mutex_init(&nvt->lifecycle_lock);
	mutex_init(&nvt->read_lock);
	init_waitqueue_head(&nvt->wait);
	ret = device_property_read_string(&spi->dev, "firmware-name", &nvt->firmware_name);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "firmware-name is required\n");
	nvt->rtx = devm_kzalloc(&spi->dev, 0x6d + 2, GFP_KERNEL);
	nvt->rrx = devm_kzalloc(&spi->dev, 0x6d + 2, GFP_KERNEL);
	if (!nvt->rtx || !nvt->rrx)
		return -ENOMEM;
	nvt->tx = kmalloc(NVT_WRITE_SIZE + 1, GFP_KERNEL);
	nvt->rx = kmalloc(NVT_READ_SIZE, GFP_KERNEL);
	nvt->record = kmalloc(sizeof(struct nvt_stream_header) + NVT_READ_SIZE, GFP_KERNEL);
	nvt->misc.name = kasprintf(GFP_KERNEL, "nvt-thp-%s", dev_name(&spi->dev));
	if (!nvt->tx || !nvt->rx || !nvt->record || !nvt->misc.name)
		return -ENOMEM;
	ret = kfifo_alloc(&nvt->fifo, SZ_256K, GFP_KERNEL);
	if (ret)
		return ret;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;
	config.max_raw_read = nvt->max_read;
	config.max_raw_write = min_t(size_t, NVT_WRITE_SIZE, transfer_max - 1);
	nvt->map = devm_regmap_init(&spi->dev, NULL, nvt, &config);
	if (IS_ERR(nvt->map))
		return PTR_ERR(nvt->map);
	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL, nvt_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, dev_name(&spi->dev), nvt);
	if (ret)
		return ret;
	spi_set_drvdata(spi, nvt);
	if (drm_is_panel_follower(&spi->dev)) {
		nvt->panel_off = true;
		nvt->follower.funcs = &nvt_panel_follower_funcs;
		ret = devm_drm_panel_add_follower(&spi->dev, &nvt->follower);
		if (ret)
			return dev_err_probe(&spi->dev, ret, "Failed to follow panel\n");
	}
	/* Boot the firmware and arm the IRQ at probe so touch works standalone. */
	{
		const struct firmware *fw;
		struct nvt_thp_firmware image;
		u16 payload_length;

		if (!request_firmware(&fw, nvt->firmware_name, &spi->dev)) {
			if (!nvt_thp_parse_firmware(fw->data, fw->size, &image)) {
				if (!nvt_thp_boot_firmware(nvt->map, fw, &image, &payload_length)) {
					WRITE_ONCE(nvt->running, true);
					set_bit(NVT_IRQ_ENABLED, &nvt->flags);
					enable_irq(spi->irq);
				}
				nvt_thp_free_firmware(&image);
			}
			release_firmware(fw);
		}
	}
	INIT_DELAYED_WORK(&nvt->wd, nvt_watchdog);
	INIT_DELAYED_WORK(&nvt->ka, nvt_keepawake_work);
	if (keepawake_ms)
		schedule_delayed_work(&nvt->ka, msecs_to_jiffies(keepawake_ms));
	nvt->input = devm_input_allocate_device(&spi->dev);
	if (!nvt->input)
		return -ENOMEM;
	nvt->input->name = "Novatek NT36526 Touchscreen";
	nvt->input->id.bustype = BUS_SPI;
	if (mt) {
		input_set_abs_params(nvt->input, ABS_MT_POSITION_X, 0, 35040, 0, 0);
		input_set_abs_params(nvt->input, ABS_MT_POSITION_Y, 0, 21900, 0, 0);
		input_set_abs_params(nvt->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
		input_mt_init_slots(nvt->input, 10, INPUT_MT_DIRECT);
		input_set_capability(nvt->input, EV_KEY, BTN_TOUCH);
	} else {
		input_set_abs_params(nvt->input, ABS_X, 0, 35040, 0, 0);
		input_set_abs_params(nvt->input, ABS_Y, 0, 21900, 0, 0);
		input_set_abs_params(nvt->input, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_capability(nvt->input, EV_KEY, BTN_TOUCH);
		input_set_capability(nvt->input, EV_KEY, BTN_LEFT);
	}
	ret = input_register_device(nvt->input);
	if (ret)
		return ret;

	/*
	 * Pen (second client of the TDDI part).  Ranges and keys follow the
	 * vendor's nvt_pen_input_config(): 12-bit pressure, hover distance,
	 * tilt +-60 degrees, eraser and two side buttons.  The vendor advertises
	 * the raw 21900 x 35040 portrait panel, but this transport applies the
	 * same axis transform as the touch device, so the pen axes are the
	 * rotated 35040 x 21900 space.
	 */
	nvt->pen_input = devm_input_allocate_device(&spi->dev);
	if (!nvt->pen_input)
		return -ENOMEM;
	nvt->pen_input->name = "NVT Capacitive Pen";
	nvt->pen_input->id.bustype = BUS_SPI;
	input_set_abs_params(nvt->pen_input, ABS_X, 0, 35040, 0, 0);
	input_set_abs_params(nvt->pen_input, ABS_Y, 0, 21900, 0, 0);
	/*
	 * libinput drops a tablet device whose ABS_X/ABS_Y carry no resolution
	 * ("missing tablet capabilities: resolution"), which is why the pen only
	 * worked through a raw evtest read and never in the desktop.  The panel
	 * is 239 x 149 mm (DRM connector size), so 35040/239 and 21900/149 both
	 * come out at ~147 units/mm.
	 */
	input_abs_set_res(nvt->pen_input, ABS_X, 147);
	input_abs_set_res(nvt->pen_input, ABS_Y, 147);
	input_set_abs_params(nvt->pen_input, ABS_PRESSURE, 0, 4095, 0, 0);
	input_set_abs_params(nvt->pen_input, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(nvt->pen_input, ABS_TILT_X, -60, 60, 0, 0);
	input_set_abs_params(nvt->pen_input, ABS_TILT_Y, -60, 60, 0, 0);
	input_set_capability(nvt->pen_input, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(nvt->pen_input, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(nvt->pen_input, EV_KEY, BTN_TOUCH);
	input_set_capability(nvt->pen_input, EV_KEY, BTN_STYLUS);
	input_set_capability(nvt->pen_input, EV_KEY, BTN_STYLUS2);
	set_bit(INPUT_PROP_DIRECT, nvt->pen_input->propbit);
	ret = input_register_device(nvt->pen_input);
	if (ret)
		return ret;
	/*
	 * The firmware was booted above, so apply the restore sequence now too.
	 * Otherwise a boot that never goes through a panel cycle keeps the
	 * firmware's default pen state (0x7b 00) and the pen packet never
	 * appears until the first blank/unblank.
	 */
	if (READ_ONCE(nvt->running)) {
		struct nvt_thp_command_status last;

		if (nvt_thp_apply_host_commands(nvt->map, &last))
			dev_warn(&spi->dev,
				 "THP host command %02x failed at probe: %d\n",
				 last.sub_command, last.status);
	}
	schedule_delayed_work(&nvt->wd, msecs_to_jiffies(3000));
	nvt->misc.minor = MISC_DYNAMIC_MINOR;
	nvt->misc.fops = &nvt_fops;
	nvt->misc.parent = &spi->dev;
	nvt->misc.mode = 0600;
	return misc_register(&nvt->misc);
}

static void nvt_remove(struct spi_device *spi)
{
	struct nvt_thp *nvt = spi_get_drvdata(spi);

	/* Do not hold lifecycle_lock while misc_open may be waiting for it. */
	misc_deregister(&nvt->misc);
	cancel_delayed_work_sync(&nvt->wd);
	mutex_lock(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->dead, true);
	nvt_stop_irq(nvt);
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->lifecycle_lock);
}

static void nvt_shutdown(struct spi_device *spi)
{
	struct nvt_thp *nvt = spi_get_drvdata(spi);

	mutex_lock(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->dead, true);
	nvt_stop_irq(nvt);
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->lifecycle_lock);
}

static const struct of_device_id nvt_of_match[] = {
	{ .compatible = "novatek,nt36526e" },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_of_match);

static const struct spi_device_id nvt_ids[] = {
	{ "nt36526e" },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_ids);

static int nvt_suspend(struct device *dev)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&nvt->lifecycle_lock);
	ret = nvt->opened ? -EBUSY : 0;
	if (!ret)
		nvt->suspended = true;
	mutex_unlock(&nvt->lifecycle_lock);
	return ret;
}

static int nvt_resume(struct device *dev)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);

	mutex_lock(&nvt->lifecycle_lock);
	nvt->suspended = false;
	mutex_unlock(&nvt->lifecycle_lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_pm, nvt_suspend, nvt_resume);

static struct spi_driver nvt_driver = {
	.driver = {
		.name = "nt36526e-thp",
		.of_match_table = nvt_of_match,
		.pm = pm_sleep_ptr(&nvt_pm),
	},
	.probe = nvt_probe,
	.remove = nvt_remove,
	.shutdown = nvt_shutdown,
	.id_table = nvt_ids,
};
module_spi_driver(nvt_driver);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36526E_THP_KUNIT_TEST)
static void nvt_test_free_fifo(void *data)
{
	struct nvt_thp *nvt = data;

	kfifo_free(&nvt->fifo);
}

static void nvt_test_open_disabled_panel(struct kunit *test)
{
	struct nvt_thp *nvt = kunit_kzalloc(test, sizeof(*nvt), GFP_KERNEL);
	struct file file = { .f_mode = FMODE_READ };
	struct file second = { .f_mode = FMODE_READ };
	struct inode inode = {};

	KUNIT_ASSERT_NOT_NULL(test, nvt);
	mutex_init(&nvt->lifecycle_lock);
	init_waitqueue_head(&nvt->wait);
	kref_init(&nvt->ref);
	KUNIT_ASSERT_EQ(test, kfifo_alloc(&nvt->fifo, 64, GFP_KERNEL), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_free_fifo, nvt), 0);
	nvt->panel_off = true;
	nvt->sequence = 41;
	nvt->error = -EIO;
	kfifo_in(&nvt->fifo, "old", 3);
	file.private_data = &nvt->misc;
	second.private_data = &nvt->misc;

	/* No SPI device: a disabled-panel open must not try firmware I/O. */
	KUNIT_ASSERT_EQ(test, nvt_open(&inode, &file), 0);
	KUNIT_EXPECT_TRUE(test, nvt->opened);
	KUNIT_EXPECT_FALSE(test, nvt->running);
	KUNIT_EXPECT_EQ(test, nvt->sequence, 41ULL);
	KUNIT_EXPECT_TRUE(test, kfifo_is_empty(&nvt->fifo));
	KUNIT_EXPECT_EQ(test, nvt_poll(&file, NULL), (__poll_t)0);
	KUNIT_EXPECT_FALSE(test, nvt_read_ready(nvt));
	KUNIT_EXPECT_EQ(test, nvt_open(&inode, &second), -EBUSY);
	KUNIT_EXPECT_EQ(test, nvt_panel_disabling(&nvt->follower), 0);
	KUNIT_EXPECT_EQ(test, nvt_panel_disabling(&nvt->follower), 0);

	/* Removal still wakes a waiting reader, and close drops its reference. */
	nvt->dead = true;
	KUNIT_EXPECT_EQ(test, nvt_poll(&file, NULL), (__poll_t)(EPOLLERR | EPOLLHUP));
	KUNIT_EXPECT_EQ(test, nvt_release(&inode, &file), 0);
	KUNIT_EXPECT_FALSE(test, nvt->opened);
	KUNIT_EXPECT_EQ(test, kref_read(&nvt->ref), 1U);
}

static struct kunit_case nvt_panel_cases[] = {
	KUNIT_CASE(nvt_test_open_disabled_panel),
	{}
};

static struct kunit_suite nvt_panel_suite = {
	.name = "nt36526e-panel",
	.test_cases = nvt_panel_cases,
};

kunit_test_suite(nvt_panel_suite);
#endif

MODULE_DESCRIPTION("NT36526E cascade SPI host-touch transport");
MODULE_LICENSE("GPL");
