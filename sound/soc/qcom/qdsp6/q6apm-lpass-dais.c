// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021, Linaro Limited

#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "q6dsp-lpass-ports.h"
#include "q6dsp-common.h"
#include "audioreach.h"
#include "q6apm.h"
#include "q6prm.h"

#define AUDIOREACH_BE_PCM_BASE	16

struct q6apm_dai_priv_data {
	struct clk *mclk;
	struct clk *bclk;
	bool mclk_enabled, bclk_enabled;
};

struct q6apm_lpass_dai_data {
	struct q6apm_graph *graph[APM_PORT_MAX];
	bool is_port_started[APM_PORT_MAX];
	struct audioreach_module_config module_config[APM_PORT_MAX];
	struct q6apm_dai_priv_data priv[APM_PORT_MAX];
};

static void q6apm_lpass_dai_disable_clocks(struct q6apm_lpass_dai_data *dai_data, int id)
{
	if (dai_data->priv[id].mclk_enabled) {
		clk_disable_unprepare(dai_data->priv[id].mclk);
		dai_data->priv[id].mclk_enabled = false;
	}

	if (dai_data->priv[id].bclk_enabled) {
		clk_disable_unprepare(dai_data->priv[id].bclk);
		dai_data->priv[id].bclk_enabled = false;
	}
}

static void q6apm_lpass_dai_put_clocks(struct q6apm_lpass_dai_data *dai_data)
{
	int i;

	for (i = 0; i < APM_PORT_MAX; i++) {
		q6apm_lpass_dai_disable_clocks(dai_data, i);

		if (dai_data->priv[i].mclk) {
			clk_put(dai_data->priv[i].mclk);
			dai_data->priv[i].mclk = NULL;
		}
		if (dai_data->priv[i].bclk) {
			clk_put(dai_data->priv[i].bclk);
			dai_data->priv[i].bclk = NULL;
		}
	}
}

static int q6dma_set_channel_map(struct snd_soc_dai *dai,
				 unsigned int tx_num,
				 const unsigned int *tx_ch_mask,
				 unsigned int rx_num,
				 const unsigned int *rx_ch_mask)
{

	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];
	int i;

	switch (dai->id) {
	case WSA_CODEC_DMA_TX_0:
	case WSA_CODEC_DMA_TX_1:
	case WSA_CODEC_DMA_TX_2:
	case VA_CODEC_DMA_TX_0:
	case VA_CODEC_DMA_TX_1:
	case VA_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
	case TX_CODEC_DMA_TX_4:
	case TX_CODEC_DMA_TX_5:
		if (!tx_ch_mask) {
			dev_err(dai->dev, "tx slot not found\n");
			return -EINVAL;
		}

		if (tx_num > AR_PCM_MAX_NUM_CHANNEL) {
			dev_err(dai->dev, "invalid tx num %d\n",
				tx_num);
			return -EINVAL;
		}
		for (i = 0; i < tx_num; i++)
			cfg->channel_map[i] = tx_ch_mask[i];

		break;
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
	case RX_CODEC_DMA_RX_0:
	case RX_CODEC_DMA_RX_1:
	case RX_CODEC_DMA_RX_2:
	case RX_CODEC_DMA_RX_3:
	case RX_CODEC_DMA_RX_4:
	case RX_CODEC_DMA_RX_5:
	case RX_CODEC_DMA_RX_6:
	case RX_CODEC_DMA_RX_7:
		/* rx */
		if (!rx_ch_mask) {
			dev_err(dai->dev, "rx slot not found\n");
			return -EINVAL;
		}
		if (rx_num > APM_PORT_MAX_AUDIO_CHAN_CNT) {
			dev_err(dai->dev, "invalid rx num %d\n",
				rx_num);
			return -EINVAL;
		}
		for (i = 0; i < rx_num; i++)
			cfg->channel_map[i] = rx_ch_mask[i];

		break;
	default:
		dev_err(dai->dev, "%s: invalid dai id 0x%x\n",
			__func__, dai->id);
		return -EINVAL;
	}

	return 0;
}

static int q6hdmi_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];
	int channels = hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_CHANNELS)->max;
	int ret;

	cfg->bit_width = params_width(params);
	cfg->sample_rate = params_rate(params);
	cfg->num_channels = channels;
	audioreach_set_default_channel_mapping(cfg->channel_map, channels);

	switch (dai->id) {
	case DISPLAY_PORT_RX_0:
		cfg->dp_idx = 0;
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		cfg->dp_idx = dai->id - DISPLAY_PORT_RX_1 + 1;
		break;
	}

	ret = q6dsp_get_channel_allocation(channels);
	if (ret < 0)
		return ret;

	cfg->channel_allocation = ret;

	return 0;
}

static int q6dma_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];
	int channels = hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_CHANNELS)->max;

	cfg->bit_width = params_width(params);
	cfg->sample_rate = params_rate(params);
	cfg->num_channels = channels;
	audioreach_set_default_channel_mapping(cfg->channel_map, channels);

	return 0;
}

/*
 * Screen rotation for the speaker slot matrix: 0, 90, 180 or 270 degrees.
 * The four TDM slots are bottom-left, bottom-right, top-left, top-right in the
 * default landscape orientation, so the device-side MFC has to duplicate the
 * stereo stream accordingly.  Writing this parameter re-programs the matrix on
 * a running graph (the vendor DSP accepts the channel mixer coefficients as a
 * normal SET_CFG, which is what Android's PAL does on rotation changes).
 */
static unsigned int spk_rotation;
static struct q6apm_graph *spk_graphs[4];

static int q6apm_lpass_dai_spk_rotation_set(const char *val,
					    const struct kernel_param *kp)
{
	unsigned int rotation, i;
	int ret;

	ret = kstrtouint(val, 0, &rotation);
	if (ret)
		return ret;
	if (rotation % 90)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(spk_graphs); i++) {
		if (!spk_graphs[i])
			continue;
		ret = q6apm_graph_set_chmixer(spk_graphs[i], rotation % 360);
		if (ret)
			return ret;
	}

	spk_rotation = rotation % 360;
	return 0;
}

static const struct kernel_param_ops q6apm_lpass_dai_spk_rotation_ops = {
	.set = q6apm_lpass_dai_spk_rotation_set,
	.get = param_get_uint,
};
module_param_cb(spk_rotation, &q6apm_lpass_dai_spk_rotation_ops, &spk_rotation, 0644);
MODULE_PARM_DESC(spk_rotation, "speaker slot matrix rotation in degrees (0/90/180/270)");

static void q6apm_lpass_dai_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc;

	if (dai_data->is_port_started[dai->id] && dai_data->graph[dai->id]) {
		rc = q6apm_graph_stop(dai_data->graph[dai->id]);
		dai_data->is_port_started[dai->id] = false;
		if (rc < 0)
			dev_err(dai->dev, "failed to stop APM port (%d)\n", rc);
	}
	dai_data->is_port_started[dai->id] = false;

	if (dai_data->graph[dai->id]) {
		q6apm_graph_close(dai_data->graph[dai->id]);
		if (spk_graphs[dai->id % ARRAY_SIZE(spk_graphs)] == dai_data->graph[dai->id])
			spk_graphs[dai->id % ARRAY_SIZE(spk_graphs)] = NULL;
		dai_data->graph[dai->id] = NULL;
	}
}

static bool skip_graph_start = true;
module_param(skip_graph_start, bool, 0644);

/*
 * The vendor AudioReach firmware wants the *stream* sub-graph started before
 * the *device* sub-graph (Android's PAL does exactly that: graph_start on the
 * stream handle, then on the device handle).  ALSA's DPCM does it the other
 * way round, so hold the backend start back by a few milliseconds.
 */
/*
 * be_start_mode: 0 = never start TDM backends explicitly (the DSP starts them
 * through the cross-graph connection), 1 = open the device graph and start it
 * 60 ms after the stream graph (Android's order: the device graph is not even
 * opened until the stream graph is up), 2 = start immediately (upstream),
 * 3 = single-graph mode: the device sub-graph lives in the frontend graph
 * (FORCE_DEVICE_GRAPH_ID 0 in the topology), so only push the TDM media
 * format through that graph; the frontend prepare/start covers the rest.
 */
static int be_start_mode = 3;
module_param(be_start_mode, int, 0644);

/*
 * Graph-info id used for the backend DAIs.  -1 keeps upstream behaviour (the
 * DAI index, i.e. the device sub-graph has a graph of its own); 0 lets the
 * backend configure a device sub-graph that the topology put into the
 * frontend graph.
 */
static int be_graph_id = 0;
module_param(be_graph_id, int, 0644);
MODULE_PARM_DESC(be_graph_id, "graph-info id for backend DAIs (-1 = DAI index)");

MODULE_PARM_DESC(be_graph_id, "graph-info id for backend DAIs (-1 = DAI index)");

static int q6apm_lpass_dai_graph_id(struct snd_soc_dai *dai)
{
	return be_graph_id >= 0 ? be_graph_id : dai->id;
}

struct deferred_be_item {
	struct snd_soc_dai *dai;
	int stream;
};

static struct deferred_be_item deferred_be_start[128];
static DEFINE_SPINLOCK(deferred_be_lock);
static void deferred_be_start_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(deferred_be_work, deferred_be_start_work);

/*
 * Open the device graph, push the configuration the machine driver stored in
 * module_config[] and start it.  This must happen *after* the stream graph
 * runs: the cross-graph connection installed by the mixer control only ends up
 * in the device graph's GRAPH_OPEN payload, and the DSP refuses a graph that
 * links from a sub-graph which has not been started yet (Android/PAL opens the
 * device graph only after graph_start of the stream handle).
 */
static int q6apm_lpass_dai_bringup(struct snd_soc_dai *dai, int stream, bool start)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];
	int idx = dai->id;
	int graph_id = q6apm_lpass_dai_graph_id(dai);
	struct q6apm_graph *graph;
	int rc;

	if (!dai_data->graph[idx]) {
		graph = q6apm_graph_open(dai->dev, NULL, dai->dev, graph_id, stream);
		if (IS_ERR(graph)) {
			dev_err(dai->dev, "Failed to open graph (%d)\n", graph_id);
			return PTR_ERR(graph);
		}
		dai_data->graph[idx] = graph;
	}

	cfg->direction = stream;
	if (be_graph_id >= 0 && graph_id == be_graph_id && be_graph_id == 0) {
		/* device sub-graph lives in the front-end graph */
		rc = q6apm_graph_media_format_device(dai_data->graph[idx], cfg);
	} else {
		rc = q6apm_graph_media_format_pcm(dai_data->graph[idx], cfg);
	}
	if (rc) {
		dev_err(dai->dev, "Failed to set media format %d\n", rc);
		return rc;
	}

	rc = q6apm_graph_prepare(dai_data->graph[idx]);
	if (rc) {
		dev_err(dai->dev, "Failed to prepare Graph %d\n", rc);
		return rc;
	}

	if (!start)
		return 0;

	rc = q6apm_graph_start(dai_data->graph[idx]);
	if (rc) {
		dev_err(dai->dev, "Failed to start APM port %d\n", dai->id);
		return rc;
	}
	dai_data->is_port_started[idx] = true;

	return 0;
}

static void deferred_be_start_work(struct work_struct *work)
{
	unsigned long flags;
	struct deferred_be_item item;
	struct snd_soc_dai *dai;
	int i;

	for (i = 0; i < ARRAY_SIZE(deferred_be_start); i++) {
		spin_lock_irqsave(&deferred_be_lock, flags);
		item = deferred_be_start[i];
		memset(&deferred_be_start[i], 0, sizeof(item));
		spin_unlock_irqrestore(&deferred_be_lock, flags);
		dai = item.dai;
		if (!dai)
			continue;
		if (q6apm_lpass_dai_bringup(dai, item.stream, true))
			pr_err("q6apm: deferred BE bring-up failed for %d\n", i);
	}
}

static void deferred_be_schedule(struct snd_soc_dai *dai, int stream, int id)
{
	unsigned long flags;

	if (id < 0 || id >= (int)ARRAY_SIZE(deferred_be_start))
		return;
	spin_lock_irqsave(&deferred_be_lock, flags);
	deferred_be_start[id].dai = dai;
	deferred_be_start[id].stream = stream;
	spin_unlock_irqrestore(&deferred_be_lock, flags);
	schedule_delayed_work(&deferred_be_work, msecs_to_jiffies(60));
}

static int q6apm_lpass_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				   struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!dai_data->is_port_started[dai->id]) {
			if ((be_start_mode == 0 || be_start_mode == 3) &&
			    dai_data->graph[dai->id] &&
			    (dai->id >= PRIMARY_TDM_RX_0 && dai->id <= QUINARY_TDM_TX_7)) {
				dev_dbg(dai->dev, "port %d: left to the DSP graph\n", dai->id);
				dai_data->is_port_started[dai->id] = true;
				break;
			}
			if (be_start_mode == 1 &&
			    (dai->id >= PRIMARY_TDM_RX_0 && dai->id <= QUINARY_TDM_TX_7)) {
				dev_dbg(dai->dev, "port %d: bring-up deferred until the stream graph runs\n",
					 dai->id);
				deferred_be_schedule(dai, substream->stream, dai->id);
				dai_data->is_port_started[dai->id] = true;
				break;
			}
			ret = q6apm_graph_start(dai_data->graph[dai->id]);
			if (ret < 0)
				dev_err(dai->dev, "Failed to start APM port %d\n", dai->id);
			else
				dai_data->is_port_started[dai->id] = true;
		}
		break;
	default:
		break;
	}

	return ret;
}

static int q6apm_lpass_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];
	struct q6apm_graph *graph;
	int graph_id = dai->id;
	int gid = q6apm_lpass_dai_graph_id(dai);
	int rc;

	if (dai_data->is_port_started[dai->id]) {
		q6apm_graph_stop(dai_data->graph[dai->id]);
		dai_data->is_port_started[dai->id] = false;

	}

	/*
	 * Single-graph mode (be_start_mode 3): the device sub-graph lives in the
	 * frontend graph, which the frontend opens and prepares.  Only push the
	 * TDM media format here - and do it now, so the interface is configured
	 * before the frontend starts the graph.
	 */
	if (be_start_mode == 3 &&
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    dai->id >= PRIMARY_TDM_RX_0 && dai->id <= QUINARY_TDM_TX_7) {
		if (!dai_data->graph[dai->id]) {
			graph = q6apm_graph_open(dai->dev, NULL, dai->dev, gid,
						 substream->stream);
			if (IS_ERR(graph)) {
				dev_err(dai->dev, "Failed to open graph (%d)\n", gid);
				return PTR_ERR(graph);
			}
			dai_data->graph[dai->id] = graph;
		}
		cfg->direction = substream->stream;
		rc = q6apm_graph_media_format_device(dai_data->graph[dai->id], cfg);
		if (rc) {
			dev_err(dai->dev, "Failed to set media format %d\n", rc);
			q6apm_graph_close(dai_data->graph[dai->id]);
			dai_data->graph[dai->id] = NULL;
			return rc;
		}
		rc = q6apm_graph_set_chmixer(dai_data->graph[dai->id], spk_rotation);
		if (rc)
			dev_warn(dai->dev, "port %d: speaker matrix failed %d\n",
				 dai->id, rc);
		spk_graphs[dai->id % ARRAY_SIZE(spk_graphs)] = dai_data->graph[dai->id];
		dev_dbg(dai->dev, "port %d: single-graph mode, media format via graph %d\n",
			 dai->id, gid);
		return 0;
	}

	/*
	 * TDM playback backends: do not open the device graph here.  DPCM
	 * prepares the backends before it starts the frontend, but the vendor
	 * DSP only accepts a device graph whose cross-graph source sub-graph is
	 * already running, so the whole bring-up is postponed to the delayed
	 * work scheduled from the trigger (see q6apm_lpass_dai_bringup()).
	 */
	if (be_start_mode == 1 &&
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    dai_data->graph[dai->id] == NULL &&
	    dai->id >= PRIMARY_TDM_RX_0 && dai->id <= QUINARY_TDM_TX_7) {
		dev_dbg(dai->dev, "port %d: graph open deferred until the stream graph runs\n",
			 dai->id);
		return 0;
	}

	/**
	 * It is recommend to load DSP with source graph first and then sink
	 * graph, so sequence for playback and capture will be different
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && dai_data->graph[dai->id] == NULL) {
		graph = q6apm_graph_open(dai->dev, NULL, dai->dev, gid, substream->stream);
		if (IS_ERR(graph)) {
			dev_err(dai->dev, "Failed to open graph (%d)\n", gid);
			rc = PTR_ERR(graph);
			return rc;
		}
		dai_data->graph[dai->id] = graph;
	}

	cfg->direction = substream->stream;
	rc = q6apm_graph_media_format_pcm(dai_data->graph[dai->id], cfg);
	if (rc) {
		dev_err(dai->dev, "Failed to set media format %d\n", rc);
		goto err;
	}

	rc = q6apm_graph_prepare(dai_data->graph[dai->id]);
	if (rc) {
		dev_err(dai->dev, "Failed to prepare Graph %d\n", rc);
		goto err;
	}
	return 0;
err:
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		q6apm_graph_close(dai_data->graph[dai->id]);
		dai_data->graph[dai->id] = NULL;
	}
	return rc;
}

static int q6apm_lpass_dai_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct q6apm_graph *graph;
	int graph_id = dai->id;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		graph = q6apm_graph_open(dai->dev, NULL, dai->dev, graph_id, substream->stream);
		if (IS_ERR(graph)) {
			dev_err(dai->dev, "Failed to open graph (%d)\n", graph_id);
			return PTR_ERR(graph);
		}
		dai_data->graph[graph_id] = graph;
	}

	return 0;
}

static int q6i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];

	cfg->fmt = fmt;

	return 0;
}

/*
 * The LPASS bit/frame clocks are owned by the PRM service: nothing comes out
 * of the TDM pins until the AP votes for the interface clock here.  The
 * machine driver calls this with LPAIF_MI2S_BCLK and the computed bit clock
 * (rate * slots * slot_width, 6.144 MHz for 48 kHz / 4 x 32 bit) right before
 * the DSP is asked to start the backend sub-graph.
 */
static int q6i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct clk *sysclk = NULL;
	bool *enabled = NULL;
	int ret = 0;

	switch (clk_id) {
	case LPAIF_MI2S_MCLK:
		sysclk = dai_data->priv[dai->id].mclk;
		enabled = &dai_data->priv[dai->id].mclk_enabled;
		break;
	case LPAIF_MI2S_BCLK:
		sysclk = dai_data->priv[dai->id].bclk;
		enabled = &dai_data->priv[dai->id].bclk_enabled;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(dai->dev, "ar-clk: set_sysclk dai_id=%d clk_id=%d freq=%u sysclk=%s\n",
		 dai->id, clk_id, freq, sysclk ? "yes" : "NULL");

	if (sysclk) {
		ret = clk_set_rate(sysclk, freq);
		if (ret) {
			dev_err(dai->dev, "Error, Unable to set rate (%d) for sysclk %d\n",
				freq, clk_id);
			return ret;
		}

		if (*enabled)
			return 0;

		ret = clk_prepare_enable(sysclk);
		if (ret) {
			dev_err(dai->dev, "Error, Unable to prepare (%d) sysclk\n", clk_id);
			return ret;
		}

		*enabled = true;
		dev_dbg(dai->dev, "ar-clk: enabled, rate now %lu\n",
			 clk_get_rate(sysclk));
	}

	return ret;
}

static void q6i2s_lpass_dai_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);

	q6apm_lpass_dai_shutdown(substream, dai);
	q6apm_lpass_dai_disable_clocks(dai_data, dai->id);
}

/*
 * TDM BEs (secondary/tertiary/... TDM) are fed by the AudioReach graph just
 * like I2S/MI2S ones, but they also need the slot layout, which the machine
 * driver hands over through snd_soc_dai_set_tdm_slot().  Without ops of their
 * own these DAIs never run prepare/hw_params, so the DSP never starts the
 * backend sub-graph and the LPASS puts no clock/data on the TDM lines.
 */
static int q6tdm_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
			      unsigned int rx_mask, int slots, int slot_width)
{
	struct q6apm_lpass_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct audioreach_module_config *cfg = &dai_data->module_config[dai->id];

	if (slots < 0 || slot_width < 0)
		return -EINVAL;

	cfg->slot_mask = tx_mask | rx_mask;
	cfg->nslots_per_frame = slots;
	cfg->slot_width = slot_width;

	return 0;
}

static const struct snd_soc_dai_ops q6tdm_ops = {
	.prepare	= q6apm_lpass_dai_prepare,
	.startup	= q6apm_lpass_dai_startup,
	.shutdown	= q6i2s_lpass_dai_shutdown,
	.set_channel_map  = q6dma_set_channel_map,
	.hw_params        = q6dma_hw_params,
	.set_fmt	= q6i2s_set_fmt,
	.set_tdm_slot	= q6tdm_set_tdm_slot,
	.set_sysclk	= q6i2s_set_sysclk,
	.trigger	= q6apm_lpass_dai_trigger,
};

static const struct snd_soc_dai_ops q6dma_ops = {
	.prepare	= q6apm_lpass_dai_prepare,
	.startup	= q6apm_lpass_dai_startup,
	.shutdown	= q6apm_lpass_dai_shutdown,
	.set_channel_map  = q6dma_set_channel_map,
	.hw_params        = q6dma_hw_params,
	.trigger	= q6apm_lpass_dai_trigger,
};

static const struct snd_soc_dai_ops q6i2s_ops = {
	.prepare	= q6apm_lpass_dai_prepare,
	.startup	= q6apm_lpass_dai_startup,
	.shutdown	= q6i2s_lpass_dai_shutdown,
	.set_channel_map  = q6dma_set_channel_map,
	.hw_params        = q6dma_hw_params,
	.set_fmt	= q6i2s_set_fmt,
	.set_sysclk	= q6i2s_set_sysclk,
	.trigger	= q6apm_lpass_dai_trigger,
};

static const struct snd_soc_dai_ops q6hdmi_ops = {
	.prepare	= q6apm_lpass_dai_prepare,
	.startup	= q6apm_lpass_dai_startup,
	.shutdown	= q6apm_lpass_dai_shutdown,
	.hw_params	= q6hdmi_hw_params,
	.set_fmt	= q6i2s_set_fmt,
	.trigger	= q6apm_lpass_dai_trigger,
};

static const struct snd_soc_component_driver q6apm_lpass_dai_component = {
	.name = "q6apm-be-dai-component",
	.of_xlate_dai_name = q6dsp_audio_ports_of_xlate_dai_name,
	.be_pcm_base = AUDIOREACH_BE_PCM_BASE,
	.use_dai_pcm_id = true,
	.remove_order   = SND_SOC_COMP_ORDER_FIRST,
};

static int of_q6apm_parse_dai_data(struct device *dev,
				   struct q6apm_lpass_dai_data *data)
{
	int ret;

	for_each_child_of_node_scoped(dev->of_node, node) {
		struct q6apm_dai_priv_data *priv;
		int id;

		ret = of_property_read_u32(node, "reg", &id);
		if (ret || id < 0 || id >= APM_PORT_MAX) {
			dev_err(dev, "valid dai id not found:%d\n", ret);
			continue;
		}

		switch (id) {
		/* MI2S/TDM specific properties */
		case PRIMARY_MI2S_RX ... QUATERNARY_MI2S_TX:
		case QUINARY_MI2S_RX ... QUINARY_MI2S_TX:
		case SENARY_MI2S_RX ... SENARY_MI2S_TX:
		case PRIMARY_TDM_RX_0 ... QUINARY_TDM_TX_7:
			priv = &data->priv[id];
			priv->mclk = of_clk_get_by_name(node, "mclk");
			if (IS_ERR(priv->mclk)) {
				int err = PTR_ERR(priv->mclk);

				priv->mclk = NULL;
				if (err == -EPROBE_DEFER) {
					q6apm_lpass_dai_put_clocks(data);
					return dev_err_probe(dev, err,
							     "unable to get mi2s mclk\n");
				}
			}

			priv->bclk = of_clk_get_by_name(node, "bclk");
			if (IS_ERR(priv->bclk)) {
				int err = PTR_ERR(priv->bclk);

				priv->bclk = NULL;
				if (err == -EPROBE_DEFER) {
					q6apm_lpass_dai_put_clocks(data);
					return dev_err_probe(dev, err,
							     "unable to get mi2s bclk\n");
				}
			}
			break;
		default:
			break;
		}
	}

	return 0;
}

static void q6apm_lpass_dai_clocks_action(void *data)
{
	q6apm_lpass_dai_put_clocks(data);
}

static int q6apm_lpass_dai_dev_probe(struct platform_device *pdev)
{
	struct q6dsp_audio_port_dai_driver_config cfg;
	struct q6apm_lpass_dai_data *dai_data;
	struct snd_soc_dai_driver *dais;
	struct device *dev = &pdev->dev;
	int num_dais;
	int ret;

	dai_data = devm_kzalloc(dev, sizeof(*dai_data), GFP_KERNEL);
	if (!dai_data)
		return -ENOMEM;

	dev_set_drvdata(dev, dai_data);

	memset(&cfg, 0, sizeof(cfg));
	cfg.q6i2s_ops = &q6i2s_ops;
	cfg.q6dma_ops = &q6dma_ops;
	cfg.q6hdmi_ops = &q6hdmi_ops;
	cfg.q6tdm_ops = &q6tdm_ops;

	ret = of_q6apm_parse_dai_data(dev, dai_data);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, q6apm_lpass_dai_clocks_action, dai_data);
	if (ret)
		return ret;

	dais = q6dsp_audio_ports_set_config(dev, &cfg, &num_dais);
	if (!dais)
		return -EINVAL;

	return devm_snd_soc_register_component(dev, &q6apm_lpass_dai_component, dais, num_dais);
}

#ifdef CONFIG_OF
static const struct of_device_id q6apm_lpass_dai_device_id[] = {
	{ .compatible = "qcom,q6apm-lpass-dais" },
	{},
};
MODULE_DEVICE_TABLE(of, q6apm_lpass_dai_device_id);
#endif

static struct platform_driver q6apm_lpass_dai_platform_driver = {
	.driver = {
		.name = "q6apm-lpass-dais",
		.of_match_table = of_match_ptr(q6apm_lpass_dai_device_id),
	},
	.probe = q6apm_lpass_dai_dev_probe,
};
module_platform_driver(q6apm_lpass_dai_platform_driver);

MODULE_DESCRIPTION("AUDIOREACH APM LPASS dai driver");
MODULE_LICENSE("GPL");
