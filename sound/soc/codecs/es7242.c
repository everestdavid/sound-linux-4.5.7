/*
 * ALSA SoC ES7242 adc driver
 *
 * Author:      David Yang, <yangxiaohua@everest-semi.com>
 *												or 
 *												  <info@everest-semi.com>
 * Copyright:   (C) 2017 Everest Semiconductor Co Ltd.,
 *
 * Based on sound/soc/codecs/wm8731.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  ES7242 is a stereo ADC of Everest
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/initval.h>

#include "es7242.h"

#define ES7242_TDM_ENABLE  1

/* codec private data */
struct es7242_priv {
	struct regmap *regmap;
	unsigned int dmic_amic;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	unsigned int tdm;
};

/*
 * ES7242 register cache
 */
static const u8 es7242_reg[] = {
	0x00, 0x00, 0x10, 0x04,	/* 0 */
	0x02, 0x13, 0x00, 0x3f,	/* 4 */
	0x11, 0x00, 0xc0, 0xc0,	/* 8 */
	0x12, 0xa0, 0x40, 			/* 12 */
};



/* Aute mute thershold */        
static const char *threshold_txt[] = {
	"-96dB",
	"-84dB", 
	"-72dB", 
	"-60dB"
	};
static const struct soc_enum automute_threshold =
	SOC_ENUM_SINGLE(ES7242_MUTECTL_REG05, 1, 4, threshold_txt);

/* Analog Input gain */
static const char *pga_txt[] = {"not allowed","0dB", "6dB"};
static const struct soc_enum pga_gain =
	SOC_ENUM_SINGLE(ES7242_ANACTL1_REG08, 4, 3, pga_txt);

/* Speed Mode Selection */
static const char *speed_txt[] = {
	"Single Speed Mode",
	"Double Speed Mode", 
	"Quard Speed Mode"
	};
static const struct soc_enum speed_mode =
	SOC_ENUM_SINGLE(ES7242_MODECFG_REG00, 2, 3, speed_txt);

static const struct snd_kcontrol_new es7242_snd_controls[] = {
	SOC_ENUM("Input PGA", pga_gain),
	SOC_SINGLE("ADC Mute", ES7242_MUTECTL_REG05, 3, 1, 0),
	SOC_SINGLE("AutoMute Enable", ES7242_MUTECTL_REG05, 0, 1, 1),
	SOC_ENUM("AutoMute Threshold", automute_threshold),
	SOC_SINGLE("TRI State Output", ES7242_STATECTL_REG06, 7, 1, 0),	
	SOC_SINGLE("MCLK Disable", ES7242_STATECTL_REG06, 6, 1, 0),
	SOC_SINGLE("Reset ADC Digital", ES7242_STATECTL_REG06, 3, 1, 0),
	SOC_SINGLE("Reset All Digital", ES7242_STATECTL_REG06, 4, 1, 0),
	SOC_SINGLE("Master Mode", ES7242_MODECFG_REG00, 1, 1, 0),
	SOC_SINGLE("Software Mode", ES7242_MODECFG_REG00, 0, 1, 0),
	SOC_ENUM("Speed Mode", speed_mode),
	SOC_SINGLE("High Pass Filter Disable", ES7242_MODECFG_REG00, 4, 1, 0),
	SOC_SINGLE("TDM Mode", ES7242_SDPFMT_REG01, 7, 1, 0),
	SOC_SINGLE("BCLK Invertor", ES7242_SDPFMT_REG01, 6, 1, 0),	
	SOC_SINGLE("LRCK Polarity Set", ES7242_SDPFMT_REG01, 5, 1, 0),
	SOC_SINGLE("Analog Power down", ES7242_ANACTL2_REG09, 7, 1, 0),
};

/* Analog Input MUX */
static const char * const es7242_analog_in_txt[] = {
		"Line1",
		"Line2",
		"Line3",
		"Line4"
		};
static const unsigned int es7242_analog_in_values[] = {
		1,
		2,
		4,
		8
		};		
static const struct soc_enum es7242_analog_input_enum =
        SOC_VALUE_ENUM_SINGLE(ES7242_ANACTL1_REG08, 0, 15,
                              ARRAY_SIZE(es7242_analog_in_txt),
                               es7242_analog_in_txt,
                               es7242_analog_in_values);
static const struct snd_kcontrol_new es7242_analog_in_mux_controls =
         SOC_DAPM_ENUM("Route", es7242_analog_input_enum);

static const struct snd_soc_dapm_widget es7242_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("Line Input 1"),
	SND_SOC_DAPM_INPUT("Line Input 2"),
	SND_SOC_DAPM_INPUT("Line Input 3"),
	SND_SOC_DAPM_INPUT("Line Input 4"),	

	SND_SOC_DAPM_MUX("Line Mux", SND_SOC_NOPM, 0, 0, &es7242_analog_in_mux_controls),
	
	SND_SOC_DAPM_PGA("Left PGA", ES7242_ANACTL0_REG07, 1, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right PGA", ES7242_ANACTL0_REG07, 0, 1, NULL, 0),
	
	SND_SOC_DAPM_ADC("Left ADC", "Capture", ES7242_ANACTL0_REG07, 3, 1),
	SND_SOC_DAPM_ADC("Right ADC", "Capture", ES7242_ANACTL0_REG07, 2, 1),
	
	SND_SOC_DAPM_AIF_OUT("I2S OUT", "I2S1 Stream",  4,
			ES7242_MUTECTL_REG05, 3, 1),
};

static const struct snd_soc_dapm_route es7242_intercon[] = {
	/* Input Mux*/
	{"Line Mux", "Line1", "Line Input 1"},
	{"Line Mux", "Line2", "Line Input 2"},
	{"Line Mux", "Line3", "Line Input 3"},
	{"Line Mux", "Line4", "Line Input 4"},

	/* input pga */
	{"Left PGA", NULL, "Line Mux"},
	{"Right PGA", NULL, "Line Mux"},
	/* ADC */
	{"Left ADC", NULL, "Left PGA"},
	{"Right ADC", NULL, "Right PGA"},
	/* I2S stream */
	{"I2S OUT", NULL, "Left ADC"},
	{"I2S OUT", NULL, "Right ADC"},
};

struct _coeff_div {
	u32 mclk;       //mclk frequency
	u32 sr_rate;       //sample rate
	u8 speedmode;				//speed mode,0=single,1=double,2=quad
	u8 adc_clk_div;         //adcclk and dacclk divider
	u8 lrckdiv;      //adclrck divider and daclrck divider
	u8 bclkdiv;          //sclk divider
	u8 osr;         //adc osr
};


/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 12.288MHZ */
	{12288000, 8000  , 0, 0x0c , 0x60, 24, 32},
	{12288000, 12000 , 0, 0x08 , 0x40, 16, 32},
	{12288000, 16000 , 0, 0x06 , 0x30, 12, 32},
	{12288000, 24000 , 0, 0x04 , 0x20, 8 , 32},
	{12288000, 32000 , 0, 0x03 , 0x18, 6 , 32},
	{12288000, 48000 , 0, 0x02 , 0x10, 4 , 32},
  {12288000, 64000 , 1, 0x03 , 0x0c, 3 , 32},  
  {12288000, 96000 , 1, 0x02 , 0x08, 2 , 32},   
	/* 11.2896MHZ */
	{11289600, 11025 , 0, 0x08 , 0x40, 16, 32},
	{11289600, 22050 , 0, 0x04 , 0x20, 8 , 32},
	{11289600, 44100 , 0, 0x02 , 0x10, 4 , 32},
	{11289600, 88200 , 1, 0x02 , 0x08, 2 , 32},

	/* 12.000MHZ */
	{12000000, 8000  , 0, 0x0c , 0xbc, 30, 31},
	{12000000, 11025 , 0, 0x08 , 0x44, 17, 34},	
	{12000000, 12000 , 0, 0x08 , 0xaa, 20, 31},
	{12000000, 16000 , 0, 0x06 , 0x9e, 15, 31},
	{12000000, 22050 , 0, 0x04 , 0x22, 8 , 34},	
	{12000000, 24000 , 0, 0x04 , 0x94, 10, 31},
	{12000000, 32000 , 0, 0x03 , 0x8a, 5 , 31},
	{12000000, 44100 , 0, 0x02 , 0x11, 4 , 34},	
	{12000000, 48000 , 0, 0x02 , 0x85, 5 , 31},
  {12000000, 96000 , 1, 0x02 , 0x85, 1 , 31}, 
};
static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].sr_rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count	= ARRAY_SIZE(rates_12288),
	.list	= rates_12288,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count	= ARRAY_SIZE(rates_112896),
	.list	= rates_112896,
};

static unsigned int rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count	= ARRAY_SIZE(rates_12),
	.list	= rates_12,
};

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es7242_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es7242_priv *es7242 = snd_soc_codec_get_drvdata(codec);

	DBG("Enter::%s----%d, freq:%ld\n",__FUNCTION__,__LINE__, freq);
		
	switch (freq) {
	case 11289600:
	case 22579200:
		es7242->sysclk_constraints = &constraints_112896;
		es7242->sysclk = freq;
		return 0;

	case 12288000:
	case 24576000:
		es7242->sysclk_constraints = &constraints_12288;
		es7242->sysclk = freq;
		return 0;

	case 12000000:
	case 24000000:
		es7242->sysclk_constraints = &constraints_12;
		es7242->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int es7242_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
  struct snd_soc_codec *codec = codec_dai->codec;
    u8 iface = 0;
		u8 adciface = 0;
    alsa_dbg("%s----%d, fmt[%02x]\n",__FUNCTION__,__LINE__,fmt);

    adciface    = snd_soc_read(codec, ES7242_SDPFMT_REG01);
		iface    = snd_soc_read(codec, ES7242_MODECFG_REG00);

    /* set master/slave audio interface */
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBM_CFM:    // MASTER MODE
        	  alsa_dbg("es7242 in master mode");
            iface |= 0x02;
            break;
        case SND_SOC_DAIFMT_CBS_CFS:    // SLAVE MODE
        	  alsa_dbg("es7242 in slave mode");
            iface &= 0xfd;
            break;
        default:
            return -EINVAL;
    }


    /* interface format */

    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_I2S:
            adciface &= 0xFC;                   
            break;
        case SND_SOC_DAIFMT_RIGHT_J:
            return -EINVAL;
        case SND_SOC_DAIFMT_LEFT_J:
            adciface &= 0xFC;                   
            adciface |= 0x01;                             
            break;
        case SND_SOC_DAIFMT_DSP_A:
            adciface &= 0xDC;                   
            adciface |= 0x03;                   
            break;
        case SND_SOC_DAIFMT_DSP_B:
            adciface &= 0xDC;                   
            adciface |= 0x23;                   
            break;
        default:
            return -EINVAL;
    }

    /* clock inversion */
    adciface &= 0xbF; 
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
        case SND_SOC_DAIFMT_NB_NF:  
            adciface &= 0xbF;                  		
            break;
        case SND_SOC_DAIFMT_IB_IF:          
            adciface |= 0x60;            
            break;
        case SND_SOC_DAIFMT_IB_NF:        
            adciface |= 0x40;           
            break;
        case SND_SOC_DAIFMT_NB_IF:
            adciface |= 0x20;            
            break;
        default:
            return -EINVAL;
    }
    snd_soc_update_bits(codec, ES7242_MODECFG_REG00, 0x02, iface);
    snd_soc_update_bits(codec, ES7242_SDPFMT_REG01, 0x03, adciface);
    return 0;
}

static int es7242_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct es7242_priv *es7242 = snd_soc_codec_get_drvdata(codec);
        
	DBG("Enter::%s----%d  es7242->sysclk=%d\n",__FUNCTION__,__LINE__,es7242->sysclk);

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	if (!es7242->sysclk) {
		dev_err(codec->dev,
			"No MCLK configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   es7242->sysclk_constraints);

	return 0;
}
static int es7242_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{ 
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct es7242_priv *es7242 = snd_soc_codec_get_drvdata(codec);
	
	u16 osrate  =  snd_soc_read(codec, ES7242_STMOSR_REG0D) & 0xc0; 
	u16 mclkdiv = snd_soc_read(codec, ES7242_MODECFG_REG00) & 0x9f;
	u16 bclkdiv = snd_soc_read(codec, ES7242_BCKDIV_REG03) & 0xc0;
	u16 adciface = snd_soc_read(codec, ES7242_SDPFMT_REG01) & 0xE3;
	u16 speedmode = snd_soc_read(codec, ES7242_MODECFG_REG00) & 0xf3;
	u16 adcdiv   = snd_soc_read(codec, ES7242_CLKDIV_REG04) & 0xf0; 
	u16 adclrckdiv = snd_soc_read(codec, ES7242_LRCDIV_REG02); 
	int coeff;
	int retv;

	coeff = get_coeff(es7242->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(es7242->sysclk / 2, params_rate(params));
		mclkdiv |= 0x20;
	}
	if (coeff < 0) {
		coeff = get_coeff(es7242->sysclk / 3, params_rate(params));
		mclkdiv &= 0x9f;
		mclkdiv |= 0x40;
	}	
	if (coeff < 0) {
		coeff = get_coeff(es7242->sysclk / 4, params_rate(params));
		mclkdiv &= 0x9f;
		mclkdiv |= 0x60;
	}		
	if (coeff < 0) {
		dev_err(codec->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8316->sysclk);
		return coeff;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adciface |= 0x000C;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adciface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adciface |= 0x0010;
		break;
	}

	/* set iface & srate*/
	snd_soc_update_bits(codec, ES7242_SDPFMT_REG01, 0x1c, adciface);
 	snd_soc_update_bits(codec, ES7242_MODECFG_REG00, 0x60, mclkdiv);
 	
	if (coeff >= 0) {
		osrate = coeff_div[coeff].osr;
		osrate &= 0x3f;
		
		bclkdiv |= coeff_div[coeff].bclkdiv;
		bclkdiv &= 0x3f;
	
		adcdiv |= coeff_div[coeff].adc_clk_div;
		adcdiv &= 0x0f;
		
		adclrckdiv |= coeff_div[coeff].lrckdiv;
		

		snd_soc_update_bits(codec, ES7242_STMOSR_REG0D, 0x3f, osrate);
		snd_soc_update_bits(codec, ES7242_BCKDIV_REG03, 0x3f, bclkdiv);
		snd_soc_update_bits(codec, ES7242_CLKDIV_REG04, 0x0f, adcdiv);
		snd_soc_update_bits(codec, ES7242_LRCDIV_REG02, 0xff, adclrckdiv);
	}

	return 0;
}

static int es7242_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "%s %d\n", __func__, mute);
	if (mute) 
	{		
		snd_soc_update_bits(codec, ES7242_MUTECTL_REG05, 0x08, 0x08);
	} 
	else 
	{
		snd_soc_update_bits(codec, ES7242_MUTECTL_REG05, 0x08, 0x00);
	}
	return 0;
}

static int es7242_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{        
	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(codec->dev, "%s on\n", __func__);
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev, "%s prepare\n", __func__);
		snd_soc_update_bits(codec, ES7242_MUTECTL_REG05, 0x08, 0x00);
		msleep(50);
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(codec->dev, "%s standby\n", __func__);
		snd_soc_update_bits(codec, ES7242_MUTECTL_REG05, 0x08, 0x08);
		snd_soc_update_bits(codec, ES7242_STATECTL_REG06, 0x40, 0x00);
		msleep(50);
		snd_soc_update_bits(codec, ES7242_ANACTL0_REG07, 0x0f, 0x00);
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(codec->dev, "%s off\n", __func__);
		snd_soc_update_bits(codec, ES7242_MUTECTL_REG05, 0x08, 0x08);
		msleep(50);
		snd_soc_update_bits(codec, ES7242_ANACTL0_REG07, 0x0f, 0x0f);
		msleep(50);
		snd_soc_update_bits(codec, ES7242_STATECTL_REG06, 0x40, 0x40);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define es7242_RATES SNDRV_PCM_RATE_8000_96000

#define es7242_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops es7242_ops = {
	.startup = es7242_pcm_startup,
	.hw_params = es7242_pcm_hw_params,
	.set_fmt = es7242_set_dai_fmt,
	.set_sysclk = es7242_set_dai_sysclk,
	.digital_mute = es7242_mute,
};

static struct snd_soc_dai_driver es7242_dai = {
	.name = "ES7242 HiFi",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7242_RATES,
		.formats = es7242_FORMATS,
	 },
	.ops = &es7242_ops,
	.symmetric_rates = 1,
};
static int es7242_suspend(struct snd_soc_codec *codec)
{
	es7242_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int es7242_resume(struct snd_soc_codec *codec)
{
	snd_soc_cache_sync(codec);
	es7242_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int es7242_probe(struct snd_soc_codec *codec)
{
	struct es7242_priv *es7242 = snd_soc_codec_get_drvdata(codec);
	int ret;
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	snd_soc_write(codec, ES7242_MODECFG_REG00, 0x01)//enter into hardware mode

	snd_soc_write(codec, ES7242_STATECTL_REG06, 0x18); //soft reset codec
	if(es7242->tdm)
		snd_soc_write(codec, ES7242_SDPFMT_REG01, 0x8F); //dsp for tdm mode
	else
		snd_soc_write(codec, ES7242_SDPFMT_REG01, 0x00); //i2s mode
		
	snd_soc_write(codec, ES7242_LRCDIV_REG02, 0x10); 
	snd_soc_write(codec, ES7242_BCKDIV_REG03, 0x04); 
	snd_soc_write(codec, ES7242_CLKDIV_REG04, 0x02); 
	snd_soc_write(codec, ES7242_MUTECTL_REG05, 0x1a); 
	snd_soc_write(codec, ES7242_ANACTL1_REG08, 0x11);   //select LINE IN1 for adc record
	snd_soc_write(codec, ES7242_ANACTL2_REG09, 0x3F); 
	snd_soc_write(codec, ES7242_STATECTL_REG06, 0x00); 
	snd_soc_write(codec, ES7242_ANACTL0_REG07, 0x00); //power up adc and analog input
	snd_soc_write(codec, ES7242_MUTECTL_REG05, 0x12); 
	/* power on device */
	es7242_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	codec->dapm.idle_bias_off = 1;
	snd_soc_add_codec_controls(codec, es7242_snd_controls,
				ARRAY_SIZE(es7242_snd_controls));

	return 0;
}

static int es7242_remove(struct snd_soc_codec *codec)
{
	es7242_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es7242 = {
	.reg_cache_size = ARRAY_SIZE(es7242_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = es7242_reg,
	.probe = es7242_probe,
	.remove = es7242_remove,
	.suspend = es7242_suspend,
	.resume = es7242_resume,
	.set_bias_level = es7242_set_bias_level,
	.dapm_widgets = es7242_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es7242_dapm_widgets),
	.dapm_routes = es7242_intercon,
	.num_dapm_routes = ARRAY_SIZE(es7242_intercon),
};

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
static int es7242_i2c_probe(struct i2c_client *i2c,
				   const struct i2c_device_id *i2c_id)
{
	struct es7242_priv *es7242;
	int ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	es7242 = devm_kzalloc(&i2c->dev, sizeof(struct es7242), GFP_KERNEL);
	if (es7242 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es7242);
	es7242->tdm =  ES7242_TDM_ENABLE;  //to initialize tdm mode
	
	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_es7242, &es7242_dai, 1);
	return ret;
}
static int __exit es7242_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id es7242_id[] = {
	{"es7242", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, es7242_id);

static struct i2c_driver es7242_i2c_driver = {
	.driver = {
		   .name = "es7242-audio-adc",
		   },
	.probe = es7242_i2c_probe,
	.remove = __exit_p(es7242_i2c_remove),
	.id_table = es7242_id,
};

#endif

static int __init es7242_modinit(void)
{
	int ret;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&es7242_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register TLV320AIC23 I2C driver: %d\n",
		       ret);
	}
#endif
	return ret;
}
module_init(es7242_modinit);

static void __exit es7242_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&es7242_i2c_driver);
#endif
}
module_exit(es7242_exit);

MODULE_DESCRIPTION("ASoC ES7242 audio adc driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com> / info@everest-semi.com");
MODULE_LICENSE("GPL v2");