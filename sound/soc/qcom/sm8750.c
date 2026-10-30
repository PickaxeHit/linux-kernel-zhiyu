// SPDX-License-Identifier: GPL-2.0
/*
 * sm8750 ASoC machine driver (AudioReach / q6apm based)
 *
 * Stage 1: a single APM <-> dummy-codec link so that the card enumerates and
 * the q6apm front-end can be exercised before the real codecs (WCD939x for
 * the headset, AW882xx for the four speakers) are wired up.
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include "qdsp6/q6prm.h"

/*
 * The q6apm-dai component registers no DAIs of its own (it is a DPCM
 * front-end: only the platform side carries the PCM callbacks), so the CPU
 * and codec sides use the dummy DAI and the real work goes through the
 * platform component referenced by OF node.
 */
static const struct snd_soc_component_driver sm8750_fe_component = {
	.name = "sm8750-apm",
};

/*
 * Front-end DAI: its .id is used by q6apm-dai as the AudioReach graph id
 * (the topology SectionGraph index).  With the dummy DAI this is 0 and the
 * graph lookup fails with -ENODEV.
 */
static struct snd_soc_dai_driver sm8750_fe_dai[] = {
	{
		.name = "sm8750-apm-fe",
		/*
		 * DAI id is the AudioReach graph id (tplg's AR_TKN_DAI_INDEX,
		 * fed from PCM_DAI_ID); the PCM device number is link->id.
		 */
		.id = 0,
		.playback = {
			.stream_name = "APM Playback",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_min = 48000,
			.rate_max = 48000,
			/* Four TDM slots: always open the PCM as 4 channels so
			 * PipeWire mixes stereo sources onto all four speakers. */
			.channels_min = 4,
			.channels_max = 4,
		},
		.capture = {
			.stream_name = "APM Capture",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_min = 48000,
			.rate_max = 48000,
			.channels_min = 1,
			.channels_max = 4,
		},
	},
};

SND_SOC_DAILINK_DEFS(apm_playback,
		     DAILINK_COMP_ARRAY(COMP_CPU("sm8750-apm-fe")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_PLATFORM("q6apm-dai")));

/*
 * The four AW882xx smart PAs hang off the secondary TDM (TDM1) of the LPASS,
 * two per serial-data line: sound-channel gives each amp its slot
 * (@34 -> 1, @35 -> 0, @36 -> 2, @37 -> 3, per the stock firmware DT).
 */
SND_SOC_DAILINK_DEFS(tdm_be,
		     DAILINK_COMP_ARRAY(COMP_CPU("SECONDARY_TDM_RX_0")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("aw88261.2-0034", "aw882xx-aif"),
					COMP_CODEC("aw88261.2-0035", "aw882xx-aif"),
					COMP_CODEC("aw88261.2-0036", "aw882xx-aif"),
					COMP_CODEC("aw88261.2-0037", "aw882xx-aif")),
		     DAILINK_COMP_ARRAY(COMP_PLATFORM("q6apm-lpass-dais")));

/*
 * The two microphones are DMIC, clocked by the VA macro and read through the
 * LPASS codec DMA (VA_CODEC_DMA_TX_0).  The topology owns the capture graph;
 * this link only has to bind the DSP backend to the macro so the macro is
 * clocked and its DMIC inputs are powered.
 */
SND_SOC_DAILINK_DEFS(va_capture,
		     DAILINK_COMP_ARRAY(COMP_CPU("VA_CODEC_DMA_TX_0")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("7660000.codec", "va_macro_tx1")),
		     DAILINK_COMP_ARRAY(COMP_PLATFORM("q6apm-lpass-dais")));

/*
 * Vendor resources put the speakers on a 4-slot TDM frame; each amp receives
 * exactly one slot.  Configure both the LPASS side (when the DAI exposes the
 * op) and the codec side from the same numbers.
 */
static bool tdm_vote_clk = true;
module_param(tdm_vote_clk, bool, 0644);
MODULE_PARM_DESC(tdm_vote_clk, "vote the TDM bit clock through PRM");

static uint tdm_bclk_scale = 1;
module_param(tdm_bclk_scale, uint, 0644);
MODULE_PARM_DESC(tdm_bclk_scale, "multiplier applied to the voted TDM bit clock");

/* Stock DTB: qcom,tdm-max-slots = <4>; 4 amps on 2 data lines, 32-bit slots. */
#define SM8750_TDM_SLOTS	4
#define SM8750_TDM_SLOT_WIDTH	32

static const unsigned int sm8750_amp_slot[] = { 1, 0, 2, 3 };

static int sm8750_tdm_be_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *dai;
	unsigned int bclk;
	int i, ret;

	/*
	 * The smart PAs only consume clocks, so the LPASS side must drive
	 * BCK/WS (BP_FP).  This also makes the DSP TDM endpoint config use
	 * an internal frame-sync/bit-clock source.
	 */
	if (cpu_dai->id == SECONDARY_MI2S_RX) {
		/* Plain I2S on the secondary MI2S: no TDM slot programming. */
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP |
						       SND_SOC_DAIFMT_I2S |
						       SND_SOC_DAIFMT_IB_NF);
		dev_dbg(cpu_dai->dev, "MI2S set_fmt(I2S): %d\n", ret);
		bclk = params_rate(params) * params_channels(params) *
		       SM8750_TDM_SLOT_WIDTH;
	} else {
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP |
						       SND_SOC_DAIFMT_DSP_B |
						       SND_SOC_DAIFMT_IB_NF);
		if (ret && ret != -ENOTSUPP)
			dev_dbg(cpu_dai->dev, "cpu set_fmt: %d\n", ret);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0,
					       GENMASK(SM8750_TDM_SLOTS - 1, 0),
					       SM8750_TDM_SLOTS, SM8750_TDM_SLOT_WIDTH);
		if (ret && ret != -ENOTSUPP)
			dev_dbg(cpu_dai->dev, "cpu tdm slot: %d\n", ret);

		bclk = params_rate(params) * SM8750_TDM_SLOTS * SM8750_TDM_SLOT_WIDTH;
	}

	/*
	 * Vote for the LPAIF TDM bit clock through the PRM service.  Nothing
	 * appears on the TDM pins and the DSP refuses to start the backend
	 * sub-graph while this clock is off, so it has to be enabled before
	 * the graph is started.  48 kHz * 4 slots * 32 bit = 6.144 MHz.
	 */
	bclk *= tdm_bclk_scale;
	if (tdm_vote_clk) {
		ret = snd_soc_dai_set_sysclk(cpu_dai, LPAIF_MI2S_BCLK, bclk,
					     SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP)
			dev_err(cpu_dai->dev, "cpu set_sysclk(%u): %d\n", bclk, ret);
		else
			dev_dbg(cpu_dai->dev, "TDM bclk %u Hz\n", bclk);
	} else {
		dev_dbg(cpu_dai->dev, "TDM bclk vote skipped (would be %u Hz)\n", bclk);
	}

	for_each_rtd_codec_dais(rtd, i, dai) {
		unsigned int slot = sm8750_amp_slot[i % ARRAY_SIZE(sm8750_amp_slot)];

		ret = snd_soc_dai_set_fmt(dai, SND_SOC_DAIFMT_DSP_B |
					       SND_SOC_DAIFMT_IB_NF |
					       SND_SOC_DAIFMT_CBC_CFC);
		if (ret) {
			dev_err(dai->dev, "set_fmt failed: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_tdm_slot(dai, 0, BIT(slot),
					       SM8750_TDM_SLOTS,
					       SM8750_TDM_SLOT_WIDTH);
		if (ret) {
			dev_err(dai->dev, "set_tdm_slot(%u) failed: %d\n", slot, ret);
			return ret;
		}
		dev_dbg(dai->dev, "TDM slot %u of %u\n", slot, SM8750_TDM_SLOTS);
	}

	return 0;
}

static const struct snd_soc_ops sm8750_be_ops = {
	.hw_params = sm8750_tdm_be_hw_params,
};

/*
 * The wire side of the TDM link is always 48 kHz / 4 x 32-bit slots, the way
 * stock Android runs TDM-LPAIF-RX-SECONDARY and the way the four amplifiers
 * expect it.  The front end may be 16-bit stereo (it is an ordinary ALSA
 * stream); the AudioReach PCM converter and MFC in the topology do the
 * conversion, so the DSP endpoint must not inherit the front-end format.
 */
static int sm8750_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	/* Wire format: 4 slots x 32 bit (stock qcom,tdm-max-slots = <4>). */
	channels->min = channels->max = 4;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);

	return 0;
}

static struct snd_soc_dai_link sm8750_dai_links[] = {
	{
		.name = "APM Playback",
		.stream_name = "MultiMedia1 Playback",
		/* tplg's MultiMedia1 Playback owns PCM device 0 */
		.id = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		/*
		 * This link has no DAPM path to the backend (the topology owns the
		 * routing), so it used to expose a second, unusable PCM
		 * (hw:0,1 + a dead capture).  Keep the DAI for the card but do not
		 * create a PCM; hw:0,0 from the topology is the real one.
		 */
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(apm_playback),
	},
	{
		.name = "Secondary TDM0",
		.stream_name = "Secondary TDM0 Playback",
		.no_pcm = 1,
		.ops = &sm8750_be_ops,
		.be_hw_params_fixup = sm8750_be_hw_params_fixup,
		SND_SOC_DAILINK_REG(tdm_be),
	},
	{
		.name = "VA Capture",
		/*
		 * Must match the capture stream name of the device sub-graph in
		 * the topology ("<DEVICE> Capture"): that widget is what the
		 * topology's capture route connects the stream mixer to, and
		 * DPCM finds backends through exactly that DAPM path.
		 */
		.stream_name = "VA_CODEC_DMA_TX_0 Capture",
		/* pure DPCM backend: the topology's capture PCM drives it */
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(va_capture),
	},
};

/*
 * The q6apm "audio" component is what loads the AudioReach topology
 * (q6apm_audio_probe -> audioreach_tplg_init).  It is not part of any DAI
 * link, so it must be listed as an auxiliary device to get probed.
 */
static struct snd_soc_aux_dev sm8750_aux_devs[] = {
	{
		/* of_node is filled in at probe time (the GPR service node) */
		.dlc = { },
	},
};

/*
 * The AudioReach "virtual mixer" control ties the stream sub-graph to the
 * device sub-graph: its put() installs the module-to-module connection that
 * ends up in the GRAPH_OPEN payload and connects the DAPM path DPCM needs to
 * find the backend.  Nothing in userspace turns it on by default, so every PCM
 * open would fail with "no backend DAIs enabled".
 *
 * The control only exists once the topology has been loaded *and* the control
 * has been registered with the card, which happens after late_probe(), so poll
 * for it from a delayed work instead of racing the card registration.
 */
static const char * const sm8750_mixer_controls[] = {
	/* stream sub-graph -> speaker TDM device sub-graph */
	"SECONDARY_TDM_RX_0 Audio Mixer MultiMedia1",
	/* stream sub-graph -> DMIC capture device sub-graph */
	"MultiMedia3 Mixer VA_CODEC_DMA_TX_0",
};
#define SM8750_TDM_MIXER_TRIES 25
#define SM8750_TDM_MIXER_INTERVAL_MS 200

static struct snd_soc_card *sm8750_card_ptr;
static int sm8750_mixer_tries;

static void sm8750_mixer_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(sm8750_mixer_work_struct, sm8750_mixer_work);

static void sm8750_mixer_work(struct work_struct *work)
{
	struct snd_ctl_elem_value uctl = { 0 };
	struct snd_kcontrol *kctl;
	struct snd_soc_card *card = sm8750_card_ptr;
	unsigned int i;
	int c;

	if (!card)
		return;

	for (i = 0; i < ARRAY_SIZE(sm8750_mixer_controls); i++) {
		kctl = snd_soc_card_get_kcontrol(card, sm8750_mixer_controls[i]);
		if (!kctl)
			continue;

		uctl.id = kctl->id;
		for (c = 0; c < 2; c++)
			uctl.value.integer.value[c] = 1;

		if (kctl->put(kctl, &uctl) < 0)
			dev_err(card->dev, "failed to enable %s\n",
				sm8750_mixer_controls[i]);
		else
			dev_dbg(card->dev, "enabled %s\n", sm8750_mixer_controls[i]);
	}

	/* the topology registers its controls asynchronously; retry until the
	 * last one shows up */
	if (!snd_soc_card_get_kcontrol(card,
				       sm8750_mixer_controls[ARRAY_SIZE(sm8750_mixer_controls) - 1])) {
		if (++sm8750_mixer_tries < SM8750_TDM_MIXER_TRIES)
			schedule_delayed_work(&sm8750_mixer_work_struct,
					      msecs_to_jiffies(SM8750_TDM_MIXER_INTERVAL_MS));
		else
			dev_warn(card->dev, "%s: capture control not found\n", __func__);
	}
}

static int sm8750_late_probe(struct snd_soc_card *card)
{
	sm8750_card_ptr = card;
	sm8750_mixer_tries = 0;
	schedule_delayed_work(&sm8750_mixer_work_struct,
			      msecs_to_jiffies(SM8750_TDM_MIXER_INTERVAL_MS));
	return 0;
}

static struct snd_soc_card sm8750_card = {
	.name = "sm8750",
	.owner = THIS_MODULE,
	.dai_link = sm8750_dai_links,
	.num_links = ARRAY_SIZE(sm8750_dai_links),
	.aux_dev = sm8750_aux_devs,
	.num_aux_devs = ARRAY_SIZE(sm8750_aux_devs),
	.late_probe = sm8750_late_probe,
};

static int sm8750_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct device_node *np;
	int ret;

	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	*card = sm8750_card;
	card->dev = &pdev->dev;
	card->driver_name = "sm8750";

	/*
	 * The platform side must reference the q6apm DAI component by OF node:
	 * its component name is the device name (e.g. "...service@1:dais"),
	 * not the DAI name.
	 */
	{
		struct device_node *svc =
			of_find_node_by_path("/soc@0/remoteproc@6800000/glink-edge/gpr/service@1");
		if (svc) {
			sm8750_aux_devs[0].dlc.of_node = svc;
			dev_dbg(&pdev->dev, "aux q6apm component %pOF\n", svc);
		} else {
			dev_warn(&pdev->dev, "q6apm service node not found for aux dev\n");
		}
	}

	np = of_find_node_by_path("/soc@0/remoteproc@6800000/glink-edge/gpr/service@1/dais");
	if (np) {
		sm8750_dai_links[0].platforms[0].of_node = np;
		sm8750_dai_links[0].platforms[0].name = NULL;
		dev_dbg(&pdev->dev, "using q6apm platform node %pOF\n", np);
	} else {
		dev_warn(&pdev->dev, "q6apm dais node not found, platform falls back to DAI name\n");
	}

	/*
	 * The four smart PAs are bound by OF node rather than component name:
	 * the Awinic driver's component name is whatever the i2c device is
	 * called, which is not something to hard-code here.
	 */
	{
		/*
		 * The vendor driver names its DAI "aw882xx-aif-<sound-channel>"
		 * (aw882xx_append_channel_suffix), not "<bus>-<addr>".  The
		 * sound-channel values are 1/0/2/3 for @34..@37.
		 */
		static const char * const amp_dai_names[] = {
			"aw882xx-aif-1", "aw882xx-aif-0",
			"aw882xx-aif-2", "aw882xx-aif-3",
		};
		static const char * const amp_paths[] = {
			"/soc@0/geniqup@ac0000/i2c@a94000/aw882xx_smartpa@34",
			"/soc@0/geniqup@ac0000/i2c@a94000/aw882xx_smartpa@35",
			"/soc@0/geniqup@ac0000/i2c@a94000/aw882xx_smartpa@36",
			"/soc@0/geniqup@ac0000/i2c@a94000/aw882xx_smartpa@37",
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(amp_paths); i++) {
			struct device_node *amp = of_find_node_by_path(amp_paths[i]);

			if (!amp) {
				dev_warn(&pdev->dev, "amp node %s not found\n",
					 amp_paths[i]);
				continue;
			}
			sm8750_dai_links[1].codecs[i].of_node = amp;
			sm8750_dai_links[1].codecs[i].name = NULL;
			sm8750_dai_links[1].codecs[i].dai_name = amp_dai_names[i];
		}
	}

	/* BE platform: the lpass-dais component, also matched by OF node */
	if (ARRAY_SIZE(sm8750_dai_links) > 1) {
		np = of_find_node_by_path("/soc@0/remoteproc@6800000/glink-edge/gpr/service@1/bedais");
		if (np) {
			sm8750_dai_links[1].platforms[0].of_node = np;
			sm8750_dai_links[1].platforms[0].name = NULL;
			dev_dbg(&pdev->dev, "using q6apm bedais node %pOF\n", np);
		} else {
			dev_warn(&pdev->dev, "q6apm bedais node not found\n");
		}
	}

	/*
	 * The capture backend uses the same LPASS DAI component; its codec side
	 * is the VA macro, which also has to be matched by OF node because the
	 * component is named after the platform device.
	 */
	{
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(sm8750_dai_links); i++) {
			struct snd_soc_dai_link *link = &sm8750_dai_links[i];

			if (strcmp(link->name, "VA Capture"))
				continue;

			np = of_find_node_by_path("/soc@0/remoteproc@6800000/glink-edge/gpr/service@1/bedais");
			if (np) {
				link->platforms[0].of_node = np;
				link->platforms[0].name = NULL;
			}

			np = of_find_node_by_path("/soc@0/codec@7660000");
			if (np) {
				link->codecs[0].of_node = np;
				link->codecs[0].name = NULL;
				link->codecs[0].dai_name = "va_macro_tx1";
			} else {
				dev_warn(&pdev->dev, "VA macro node not found\n");
			}
		}
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &sm8750_fe_component,
					      sm8750_fe_dai,
					      ARRAY_SIZE(sm8750_fe_dai));
	if (ret) {
		dev_err(&pdev->dev, "FE component register failed %d\n", ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "registering sm8750 card\n");
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	dev_dbg(&pdev->dev, "card register returned %d\n", ret);
	return ret;
}

static const struct of_device_id snd_sm8750_dt_match[] = {
	{ .compatible = "qcom,sm8750-sndcard" },
	{}
};
MODULE_DEVICE_TABLE(of, snd_sm8750_dt_match);

static struct platform_driver snd_sm8750_driver = {
	.probe = sm8750_platform_probe,
	.driver = {
		.name = "snd-sm8750",
		.of_match_table = snd_sm8750_dt_match,
	},
};
module_platform_driver(snd_sm8750_driver);

MODULE_AUTHOR("zhiyu port");
MODULE_DESCRIPTION("Qualcomm SM8750 ASoC machine driver");
MODULE_LICENSE("GPL");
