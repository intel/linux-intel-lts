/*
 * snd_sst_tokens.h - Intel SST tokens definition
 *
 * Copyright (C) 2016 Intel Corp
 * Author: Shreyas NC <shreyas.nc@intel.com>
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#ifndef __SND_SST_TOKENS_H__
#define __SND_SST_TOKENS_H__

/**
 * %SKL_TKN_UUID:               Module UUID
 *
 * %SKL_TKN_U8_BLOCK_TYPE:      Type of the private data block.Can be:
 *                              tuples, bytes, short and words
 *
 * %SKL_TKN_U8_IN_PIN_TYPE:     Input pin type,
 *                              homogenous=0, heterogenous=1
 *
 * %SKL_TKN_U8_OUT_PIN_TYPE:    Output pin type,
 *                              homogenous=0, heterogenous=1
 * %SKL_TKN_U8_DYN_IN_PIN:      Configure Input pin dynamically
 *                              if true
 *
 * %SKL_TKN_U8_DYN_OUT_PIN:     Configure Output pin dynamically
 *                              if true
 *
 * %SKL_TKN_U8_IN_QUEUE_COUNT:  Store the number of Input pins
 *
 * %SKL_TKN_U8_OUT_QUEUE_COUNT: Store the number of Output pins
 *
 * %SKL_TKN_U8_TIME_SLOT:       TDM slot number
 *
 * %SKL_TKN_U8_CORE_ID:         Stores module affinity value.Can take
 *                              the values:
 *                              SKL_AFFINITY_CORE_0 = 0,
 *                              SKL_AFFINITY_CORE_1,
 *                              SKL_AFFINITY_CORE_MAX
 *
 * %SKL_TKN_U8_MOD_TYPE:        Module type value.
 *
 * %SKL_TKN_U8_CONN_TYPE:       Module connection type can be a FE,
 *                              BE or NONE as defined :
 *                              SKL_PIPE_CONN_TYPE_NONE = 0,
 *                              SKL_PIPE_CONN_TYPE_FE = 1 (HOST_DMA)
 *                              SKL_PIPE_CONN_TYPE_BE = 2 (LINK_DMA)
 *
 * %SKL_TKN_U8_DEV_TYPE:        Type of device to which the module is
 *                              connected
 *                              Can take the values:
 *                              SKL_DEVICE_BT = 0x0,
 *                              SKL_DEVICE_DMIC = 0x1,
 *                              SKL_DEVICE_I2S = 0x2,
 *                              SKL_DEVICE_SLIMBUS = 0x3,
 *                              SKL_DEVICE_HDALINK = 0x4,
 *                              SKL_DEVICE_HDAHOST = 0x5,
 *                              SKL_DEVICE_NONE
 *
 * %SKL_TKN_U8_HW_CONN_TYPE:    Connection type of the HW to which the
 *                              module is connected
 *                              SKL_CONN_NONE = 0,
 *                              SKL_CONN_SOURCE = 1,
 *                              SKL_CONN_SINK = 2
 *
 * %SKL_TKN_U16_PIN_INST_ID:    Stores the pin instance id
 *
 * %SKL_TKN_U16_MOD_INST_ID:    Stores the mdule instance id
 *
 * %SKL_TKN_U32_MAX_MCPS:       Module max mcps value
 *
 * %SKL_TKN_U32_MEM_PAGES:      Module resource pages
 *
 * %SKL_TKN_U32_OBS:            Stores Output Buffer size
 *
 * %SKL_TKN_U32_IBS:            Stores input buffer size
 *
 * %SKL_TKN_U32_VBUS_ID:        Module VBUS_ID. PDM=0, SSP0=0,
 *                              SSP1=1,SSP2=2,
 *                              SSP3=3, SSP4=4,
 *                              SSP5=5, SSP6=6,INVALID
 *
 * %SKL_TKN_U32_PARAMS_FIXUP:   Module Params fixup mask
 * %SKL_TKN_U32_CONVERTER:      Module params converter mask
 * %SKL_TKN_U32_PIPE_ID:        Stores the pipe id
 *
 * %SKL_TKN_U32_PIPE_CONN_TYPE: Type of the token to which the pipe is
 *                              connected to. It can be
 *                              SKL_PIPE_CONN_TYPE_NONE = 0,
 *                              SKL_PIPE_CONN_TYPE_FE = 1 (HOST_DMA),
 *                              SKL_PIPE_CONN_TYPE_BE = 2 (LINK_DMA),
 *
 * %SKL_TKN_U32_PIPE_PRIORITY:  Pipe priority value
 * %SKL_TKN_U32_PIPE_MEM_PGS:   Pipe resource pages
 *
 * %SKL_TKN_U32_DIR_PIN_COUNT:  Value for the direction to set input/output
 *                              formats and the pin count.
 *                              The first 4 bits have the direction
 *                              value and the next 4 have
 *                              the pin count value.
 *                              SKL_DIR_IN = 0, SKL_DIR_OUT = 1.
 *                              The input and output formats
 *                              share the same set of tokens
 *                              with the distinction between input
 *                              and output made by reading direction
 *                              token.
 *
 * %SKL_TKN_U32_FMT_CH:         Supported channel count
 *
 * %SKL_TKN_U32_FMT_FREQ:       Supported frequency/sample rate
 *
 * %SKL_TKN_U32_FMT_BIT_DEPTH:  Supported container size
 *
 * %SKL_TKN_U32_FMT_SAMPLE_SIZE:Number of samples in the container
 *
 * %SKL_TKN_U32_FMT_CH_CONFIG:  Supported channel configurations for the
 *                              input/output.
 *
 * %SKL_TKN_U32_FMT_INTERLEAVE: Interleaving style which can be per
 *                              channel or per sample. The values can be :
 *                              SKL_INTERLEAVING_PER_CHANNEL = 0,
 *                              SKL_INTERLEAVING_PER_SAMPLE = 1,
 *
 * %SKL_TKN_U32_FMT_SAMPLE_TYPE:
 *                              Specifies the sample type. Can take the
 *                              values: SKL_SAMPLE_TYPE_INT_MSB = 0,
 *                              SKL_SAMPLE_TYPE_INT_LSB = 1,
 *                              SKL_SAMPLE_TYPE_INT_SIGNED = 2,
 *                              SKL_SAMPLE_TYPE_INT_UNSIGNED = 3,
 *                              SKL_SAMPLE_TYPE_FLOAT = 4
 *
 * %SKL_TKN_U32_CH_MAP:         Channel map values
 * %SKL_TKN_U32_MOD_SET_PARAMS: It can take these values:
 *                              SKL_PARAM_DEFAULT, SKL_PARAM_INIT,
 *                              SKL_PARAM_SET, SKL_PARAM_BIND
 *
 * %SKL_TKN_U32_MOD_PARAM_ID:   ID of the module params
 *
 * %SKL_TKN_U32_CAPS_SET_PARAMS:
 *                              Set params value
 *
 * %SKL_TKN_U32_CAPS_PARAMS_ID: Params ID
 *
 * %SKL_TKN_U32_CAPS_SIZE:      Caps size
 *
 * %SKL_TKN_U32_PROC_DOMAIN:    Specify processing domain
 *
 * %SKL_TKN_U32_LIB_COUNT:      Specifies the number of libraries
 *
 * %SKL_TKN_STR_LIB_NAME:       Specifies the library name
 *
 * %SKL_TKN_U32_PMODE:		Specifies the power mode for pipe
 *
 * %SKL_TKL_U32_D0I3_CAPS:	Specifies the D0i3 capability for module
 *
 * %SKL_TKN_U8_PDI_TYPE:        Specifies PDI type. Can be PCM or PDM
 *
 * %SKL_TKN_U8_CONF_VERSION:    Version of topology conf file format
 *
 * %SKL_TKN_STR_PIPE_NAME:      Name of the pipe
 *
 * %SKL_TKN_STR_BE_PIPE_DEVICE: Port name when pipeline is connected to link.
 *                              Should match BE AIF widget name.
 *
 * %SKL_TKN_STR_FE_PIPE_DEVICE: Device name when pipeline is connected to
 *                              host dma.
 *                              Should match BE AIF widget name.
 *
 * %SKL_TKN_U32_PIPE_CREATE_PRIORITY:
 *                              Set this ito override the default priority
 *                              of scheduling the pipes. If the priority is
 *                              set to 0, they get scheduled in order of
 *                              creation.
 *
 * %SKL_TKN_U32_PIPE_DIR:       Specifies pipe direction. Can be
 *                              playback/capture.
 *
 * %SKL_TKN_U32_PIPE_ORDER:     Pipe order from Source to Sink.
 *
 * %SKL_TKN_U32_PIPE_LINK_TYPE: Link type between modules of the pipe.
 *                              "direct" means its statically connected,
 *                              "mixer" means Mixer user control needs to be
 *                              created to connect modules.
 *
 * %SKL_TKN_U32_PIPE_MODE:      Configuration in which pipe can be run.
 *                              Can be epmode: where pipeline FE parameters
 *                              are propogated upto the Link
 *                              or can be dspmode: pipeline is routed via
 *                              dsp and user sets the params.
 *
 * %SKL_TKN_U32_PIPE_NUM_MODULES:
 *                              Number of modules in the pipe
 *
 * %SKL_TKN_U32_NUM_CONFIGS:    Number of pipe configs
 *
 * %SKL_TKN_STR_CONFIG_NAME:    Name of the pipe config
 *
 * %SKL_TKN_U32_PIPE_CONFIG_ID: Config id for the modules in the pipe
 *                              and PCM params supported by that pipe
 *                              config. This is used as index to fill
 *                              up the pipe config and module config
 *                              structure.
 *
 * %SKL_TKN_U32_PATH_MEM_PGS:   Memory pages for the path
 *
 * %SKL_TKN_U32_CFG_FREQ:
 * %SKL_TKN_U8_CFG_CHAN:
 * %SKL_TKN_U8_CFG_BPS:         PCM params (freq, channels, bits per sample)
 *                              supported for each of the pipe configs.
 *
 * %SKL_TKN_CFG_MOD_RES_ID:     Module's resource index for each of the
 *                              pipe config
 *
 * %SKL_TKN_CFG_MOD_RES_ID:     Module's interface index for each of the
 *                              pipe config
 *
 * %SKL_TKN_U8_NUM_MOD:         Number of modules in the manifest
 *
 * %SKL_TKN_U8_LIB_IDX:         Index for library structure
 *
 * %SKL_TKN_NUM_FW_BINS:        Number of firmware binaries
 *
 * %SKL_TKN_U32_MAN_CFG_IDX:    Config index to fill up FW config info
 *                              from the manifest.
 *
 * %SKL_TKN_U32_MEM_TYPE:
 * %SKL_TKN_U32_SCH_TYPE:
 * %SKL_TKN_U32_DMA_TYPE:       Type information for TLVs
 *
 * %SKL_TKN_U32_MEM_SIZE:
 * %SKL_TKN_U32_SCH_SIZE:
 * %SKL_TKN_U32_DMA_SIZE:       Size information for TLV
 *
 * %SKL_TKN_U32_MEM_STAT_RECLAIM:
 *                              Indicates whether legacy DMA memory is
 *                              managed by DSP.
 *
 * %SKL_TKN_U32_DMA_MAX_SIZE:
 *                              Maximum DMA buffer size
 *
 * %SKL_TKN_U32_DMA_MIN_SIZE:
 *                              Minimum DMA buffer size
 *
 * %SKL_TKN_U32_SCH_TICK_MUL:   FW Scheduler tick multiplier
 *
 * %SKL_TKN_U32_SCH_TICK_DIV:   FW scheduler tick divider
 *
 * %SKL_TKN_U32_SCH_LL_SRC:     Low latency interrupt source
 *
 * %SKL_TKN_U32_SCH_NUM_CONF:   Number of configs
 *
 * %SKL_TKN_MM_U8_MAJOR_VER:    Major version of firmware extended manifest
 *
 * %SKL_TKN_MM_U8_MINOR_VER:    Minor version of firmware extended manifest
 *
 * %SKL_TKN_MM_U8_HOTFIX_VER:   Firmware version
 *
 * %SKL_TKN_MM_U8_AUTO_START:   Module instance should be created
 *                              automatically at the start of the base
 *                              firmware.
 *
 * %SKL_TKN_MM_U8_MAX_PINS:     Max in/out pins for the module
 *
 * %SKL_TKN_MM_U8_MAX_INST_COUNT:
 *                              Max allowed instance count for modules
 *
 * %SKL_TKN_MM_U8_NUM_RES:      Number of resources for the module
 *
 * %SKL_TKN_MM_U8_NUM_INTF:     Number of interfaces for the module
 *
 * %SKL_TKN_MM_U32_RES_ID:      Resource index for the resource info to
 *                              be filled into.
 *                              A module can support multiple resource
 *                              configuration and is represnted as a
 *                              resource table. This index is used to
 *                              fill information into appropriate index.
 *
 * %SKL_TKN_MM_U32_CPS:         DSP cycles
 *
 * %SKL_TKN_MM_U32_DMA_SIZE:    Allocated buffer size for gateway DMA
 *
 * %SKL_TKN_MM_U32_CPC:         DSP cycles allocated per frame
 *
 * %SKL_TKN_MM_U32_MOD_FLAGS:   Flags for the module
 *
 * %SKL_TKN_MM_U32_OBLS:        Output Block size
 *
 * %SKL_TKN_MM_U32_NUM_PIN:     Total number of input/output pins
 *
 * %SKL_TKN_MM_U32_RES_PIN_ID:  Resource pin id
 *
 * %SKL_TKN_MM_U32_INTF_PIN_ID: Pin index in the module
 *
 * %SKL_TKN_MM_U32_PIN_BUF:     Pin buffer size
 *
 * %SKL_TKN_MM_U32_FMT_ID:      Format index for each of the interface/
 *                              format information to be filled into.
 *
 * %SKL_TKN_MM_U32_NUM_IN_FMT:
 * %SKL_TKN_MM_U32_NUM_OUT_FMT: Number of input/output formats
 *
 * module_id and loadable flags dont have tokens as these values will be
 * read from the DSP FW manifest
 *
 * Tokens defined can be used either in the manifest or widget private data.
 *
 * SKL_TKN_MM is used as a suffix for all tokens that represent
 * module data in the manifest.
 */
enum SKL_TKNS {
	SKL_TKN_UUID = 1,
	SKL_TKN_U8_NUM_BLOCKS,
	SKL_TKN_U8_BLOCK_TYPE,
	SKL_TKN_U8_IN_PIN_TYPE,
	SKL_TKN_U8_OUT_PIN_TYPE,
	SKL_TKN_U8_DYN_IN_PIN,
	SKL_TKN_U8_DYN_OUT_PIN,
	SKL_TKN_U8_IN_QUEUE_COUNT,
	SKL_TKN_U8_OUT_QUEUE_COUNT,
	SKL_TKN_U8_TIME_SLOT,
	SKL_TKN_U8_CORE_ID,
	SKL_TKN_U8_MOD_TYPE,
	SKL_TKN_U8_CONN_TYPE,
	SKL_TKN_U8_DEV_TYPE,
	SKL_TKN_U8_HW_CONN_TYPE,
	SKL_TKN_U16_MOD_INST_ID,
	SKL_TKN_U16_BLOCK_SIZE,
	SKL_TKN_U32_MAX_MCPS,
	SKL_TKN_U32_MEM_PAGES,
	SKL_TKN_U32_OBS,
	SKL_TKN_U32_IBS,
	SKL_TKN_U32_VBUS_ID,
	SKL_TKN_U32_PARAMS_FIXUP,
	SKL_TKN_U32_CONVERTER,
	SKL_TKN_U32_PIPE_ID,
	SKL_TKN_U32_PIPE_CONN_TYPE,
	SKL_TKN_U32_PIPE_PRIORITY,
	SKL_TKN_U32_PIPE_MEM_PGS,
	SKL_TKN_U32_DIR_PIN_COUNT,
	SKL_TKN_U32_FMT_CH,
	SKL_TKN_U32_FMT_FREQ,
	SKL_TKN_U32_FMT_BIT_DEPTH,
	SKL_TKN_U32_FMT_SAMPLE_SIZE,
	SKL_TKN_U32_FMT_CH_CONFIG,
	SKL_TKN_U32_FMT_INTERLEAVE,
	SKL_TKN_U32_FMT_SAMPLE_TYPE,
	SKL_TKN_U32_FMT_CH_MAP,
	SKL_TKN_U32_PIN_MOD_ID,
	SKL_TKN_U32_PIN_INST_ID,
	SKL_TKN_U32_MOD_SET_PARAMS,
	SKL_TKN_U32_MOD_PARAM_ID,
	SKL_TKN_U32_CAPS_SET_PARAMS,
	SKL_TKN_U32_CAPS_PARAMS_ID,
	SKL_TKN_U32_CAPS_SIZE,
	SKL_TKN_U32_PROC_DOMAIN,
	SKL_TKN_U32_LIB_COUNT,
	SKL_TKN_STR_LIB_NAME,
	SKL_TKN_U32_PMODE,
	SKL_TKL_U32_D0I3_CAPS, /* Typo added at v4.10 */
	SKL_TKN_U32_D0I3_CAPS = SKL_TKL_U32_D0I3_CAPS,
	SKL_TKN_U8_PDI_TYPE,

	SKL_TKN_U8_CONF_VERSION,
	SKL_TKN_STR_PIPE_NAME,
	SKL_TKN_STR_PIPE_PORT,
	SKL_TKN_STR_PIPE_DEVICE,
	SKL_TKN_U32_PIPE_CREATE_PRIORITY,
	SKL_TKN_U32_PIPE_DIRECTION,
	SKL_TKN_U32_PIPE_ORDER,
	SKL_TKN_U32_PIPE_LINK_TYPE,
	SKL_TKN_U32_PIPE_MODE,
	SKL_TKN_U32_PIPE_NUM_MODULES,
	SKL_TKN_U32_PIPE_CONFIG_ID,
	SKL_TKN_U32_NUM_CONFIGS,
	SKL_TKN_STR_CONFIG_NAME,

	SKL_TKN_U32_PATH_MEM_PGS,

	SKL_TKN_U32_CFG_FREQ,
	SKL_TKN_U8_CFG_CHAN,
	SKL_TKN_U8_CFG_BPS,
	SKL_TKN_CFG_MOD_RES_ID,
	SKL_TKN_CFG_MOD_FMT_IDX,
	SKL_TKN_U8_NUM_MOD,
	SKL_TKN_U8_LIB_IDX,
	SKL_TKN_NUM_FW_BINS,

	SKL_TKN_U32_MAN_CFG_IDX,
	SKL_TKN_U32_MEM_TYPE,
	SKL_TKN_U32_MEM_SIZE,
	SKL_TKN_U32_MEM_STAT_RECLAIM,
	SKL_TKN_U32_DMA_TYPE,
	SKL_TKN_U32_DMA_SIZE,
	SKL_TKN_U32_DMA_MAX_SIZE,
	SKL_TKN_U32_DMA_MIN_SIZE,
	SKL_TKN_U32_SCH_TYPE,
	SKL_TKN_U32_SCH_SIZE,
	SKL_TKN_U32_SCH_TICK_MUL,
	SKL_TKN_U32_SCH_TICK_DIV,
	SKL_TKN_U32_SCH_LL_SRC,
	SKL_TKN_U32_SCH_NODE_INFO,
	SKL_TKN_U32_SCH_NUM_CONF,

	SKL_TKN_U8_NR_MODS,
	SKL_TKN_U8_MAJOR_VER,
	SKL_TKN_U8_MINOR_VER,
	SKL_TKN_U8_HOTFIX_VER,
	SKL_TKN_U8_BUILD_VER,
	SKL_TKN_U8_EXT_NR_MODS,
	SKL_TKN_U8_EXT_MAJOR_VER,
	SKL_TKN_U8_EXT_MINOR_VER,
	SKL_TKN_U8_PRE_LOAD_PGS,

	SKL_TKN_MM_U8_MOD_IDX,
	SKL_TKN_MM_U8_MAJOR_VER,
	SKL_TKN_MM_U8_MINOR_VER,
	SKL_TKN_MM_U8_HOTFIX_VER,
	SKL_TKN_MM_U8_BUILD_VER,
	SKL_TKN_MM_U8_AUTO_START,
	SKL_TKN_MM_U8_BINARY_TYPE,
	SKL_TKN_STR_MOD_LIB_NAME,
	SKL_TKN_MM_U8_MAX_PINS,
	SKL_TKN_MM_U8_MAX_INST_COUNT,
	SKL_TKN_MM_U8_NUM_RES,
	SKL_TKN_MM_U8_NUM_INTF,
	SKL_TKN_MM_U32_RES_ID,
	SKL_TKN_MM_U32_CPS,
	SKL_TKN_MM_U32_DMA_SIZE,
	SKL_TKN_MM_U32_CPC,
	SKL_TKN_MM_U32_MOD_FLAGS,
	SKL_TKN_MM_U32_OBLS,
	SKL_TKN_MM_U32_NUM_PIN,
	SKL_TKN_MM_U32_RES_PIN_ID,
	SKL_TKN_MM_U32_INTF_PIN_ID,
	SKL_TKN_MM_U32_PIN_BUF,
	SKL_TKN_MM_U32_FMT_ID,
	SKL_TKN_MM_U32_NUM_IN_FMT,
	SKL_TKN_MM_U32_NUM_OUT_FMT,

	SKL_TKN_MAX = SKL_TKN_MM_U32_NUM_OUT_FMT,
};

#endif
