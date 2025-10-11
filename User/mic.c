#include "User/mic.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include "MKF360_config.h"
#include "dfsdm.h"
#include "mdma.h"

__attribute__((aligned(8))) __attribute__((section(".bss.DMA_RAM_D2"))) MDMA_LinkNodeTypeDef node_mdma_channel0_sw_1;
__attribute__((aligned(8))) __attribute__((section(".bss.DMA_RAM_D2"))) MDMA_LinkNodeTypeDef node_mdma_channel0_sw_2;
__attribute__((aligned(8))) __attribute__((section(".bss.DMA_RAM_D2"))) MDMA_LinkNodeTypeDef node_mdma_channel0_sw_3;
__attribute__((aligned(8))) __attribute__((section(".bss.DMA_RAM_D2"))) MDMA_LinkNodeTypeDef node_mdma_channel1_sw_1;
__attribute__((aligned(8))) __attribute__((section(".bss.DMA_RAM_D2"))) MDMA_LinkNodeTypeDef node_mdma_channel1_sw_2;
__attribute__((aligned(8))) __attribute__((section(".bss.DMA_RAM_D2"))) MDMA_LinkNodeTypeDef node_mdma_channel1_sw_3;

__attribute__((section(".bss.DMA_RAM_D2"))) static int16_t mic_data[4][2][DFSDM_DMA_FRAME_SAMPLE_NUM];
__attribute__((section(".bss.DTCM"))) static int16_t mic_data_interlaced[4 * DFSDM_DMA_FRAME_SAMPLE_NUM];
__attribute__((section(".bss.DTCM"))) static uint8_t mic_data_interlaced_from;

__attribute__((section(".bss.DTCM"))) static MicInterlacedDataReadyIsrCallback mic_interlaced_data_ready_callback;

static void mic_data_interlace_complete(MDMA_HandleTypeDef *hmdma)
{
    (void)hmdma;
    if (hmdma == &hmdma_mdma_channel0_sw_0)
    {
        mic_data_interlaced_from = 0;
    }
    else if (hmdma == &hmdma_mdma_channel1_sw_0)
    {
        mic_data_interlaced_from = 1;
    }
    if (mic_interlaced_data_ready_callback != NULL)
    {
        mic_interlaced_data_ready_callback(&mic_data_interlaced[0], DFSDM_DMA_FRAME_SAMPLE_NUM * 4U);
    }
}

void mic_mdma_init()
{
    __HAL_RCC_MDMA_CLK_ENABLE();

    HAL_StatusTypeDef ret_hal = HAL_OK;
    MDMA_LinkNodeConfTypeDef nodeConfig = {
        .Init =
            {
                .Request = MDMA_REQUEST_SW,
                .TransferTriggerMode = MDMA_FULL_TRANSFER,
                .Priority = MDMA_PRIORITY_VERY_HIGH,
                .Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE,
                .SourceInc = MDMA_SRC_INC_HALFWORD,
                .DestinationInc = MDMA_DEST_INC_HALFWORD,
                .SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD,
                .DestDataSize = MDMA_DEST_DATASIZE_HALFWORD,
                .DataAlignment = MDMA_DATAALIGN_PACKENABLE,
                .BufferTransferLength = 2,
                .SourceBurst = MDMA_SOURCE_BURST_SINGLE,
                .DestBurst = MDMA_DEST_BURST_SINGLE,
                .SourceBlockAddressOffset = 0,
                .DestBlockAddressOffset = 6,
            },
        .PostRequestMaskAddress = 0,
        .PostRequestMaskData = 0,
        .BlockDataLength = 2,
        .BlockCount = DFSDM_DMA_FRAME_SAMPLE_NUM,
    };

    // MDMA channel 0
    // For interlace mic first half data
    hmdma_mdma_channel0_sw_0.Instance = MDMA_Channel0;
    hmdma_mdma_channel0_sw_0.Init.Request = MDMA_REQUEST_SW;
    hmdma_mdma_channel0_sw_0.Init.TransferTriggerMode = MDMA_FULL_TRANSFER;
    hmdma_mdma_channel0_sw_0.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
    hmdma_mdma_channel0_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_mdma_channel0_sw_0.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
    hmdma_mdma_channel0_sw_0.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
    hmdma_mdma_channel0_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
    hmdma_mdma_channel0_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
    hmdma_mdma_channel0_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_mdma_channel0_sw_0.Init.BufferTransferLength = 2;
    hmdma_mdma_channel0_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma_mdma_channel0_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_mdma_channel0_sw_0.Init.SourceBlockAddressOffset = 0;
    hmdma_mdma_channel0_sw_0.Init.DestBlockAddressOffset = 6;
    ret_hal = HAL_MDMA_Init(&hmdma_mdma_channel0_sw_0);
    if (ret_hal != HAL_OK)
    {
        Error_Handler();
    }

    MDMA_LinkNodeTypeDef *const mdma_ch0_node[] = {
        &node_mdma_channel0_sw_1,
        &node_mdma_channel0_sw_2,
        &node_mdma_channel0_sw_3,
    };
    int16_t *const mdma_ch0_src_address[] = {
        &(mic_data[1][0][0]),
        &(mic_data[2][0][0]),
        &(mic_data[3][0][0]),
    };
    int16_t *const mdma_ch0_dest_address[] = {
        &(mic_data_interlaced[1]),
        &(mic_data_interlaced[2]),
        &(mic_data_interlaced[3]),
    };
    for (size_t i = 0; i < 3; i++)
    {
        nodeConfig.SrcAddress = (uint32_t)mdma_ch0_src_address[i];
        nodeConfig.DstAddress = (uint32_t)mdma_ch0_dest_address[i];
        ret_hal = HAL_MDMA_LinkedList_CreateNode(mdma_ch0_node[i], &nodeConfig);
        if (ret_hal != HAL_OK)
        {
            Error_Handler();
        }
        ret_hal = HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_sw_0, mdma_ch0_node[i], 0);
        if (ret_hal != HAL_OK)
        {
            Error_Handler();
        }
    }

    // MDMA channel 1
    // For interlace mic second half data
    hmdma_mdma_channel1_sw_0.Instance = MDMA_Channel1;
    hmdma_mdma_channel1_sw_0.Init.Request = MDMA_REQUEST_SW;
    hmdma_mdma_channel1_sw_0.Init.TransferTriggerMode = MDMA_FULL_TRANSFER;
    hmdma_mdma_channel1_sw_0.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
    hmdma_mdma_channel1_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_mdma_channel1_sw_0.Init.SourceInc = MDMA_SRC_INC_HALFWORD;
    hmdma_mdma_channel1_sw_0.Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
    hmdma_mdma_channel1_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
    hmdma_mdma_channel1_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
    hmdma_mdma_channel1_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_mdma_channel1_sw_0.Init.BufferTransferLength = 2;
    hmdma_mdma_channel1_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma_mdma_channel1_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_mdma_channel1_sw_0.Init.SourceBlockAddressOffset = 0;
    hmdma_mdma_channel1_sw_0.Init.DestBlockAddressOffset = 6;
    if (HAL_MDMA_Init(&hmdma_mdma_channel1_sw_0) != HAL_OK)
    {
        Error_Handler();
    }

    MDMA_LinkNodeTypeDef *const mdma_ch1_node[] = {
        &node_mdma_channel1_sw_1,
        &node_mdma_channel1_sw_2,
        &node_mdma_channel1_sw_3,
    };
    int16_t *const mdma_ch1_src_address[] = {
        &(mic_data[1][1][0]),
        &(mic_data[2][1][0]),
        &(mic_data[3][1][0]),
    };
    int16_t *const mdma_ch1_dest_address[] = {
        &(mic_data_interlaced[1]),
        &(mic_data_interlaced[2]),
        &(mic_data_interlaced[3]),
    };
    for (size_t i = 0; i < 3; i++)
    {
        nodeConfig.SrcAddress = (uint32_t)mdma_ch1_src_address[i];
        nodeConfig.DstAddress = (uint32_t)mdma_ch1_dest_address[i];
        ret_hal = HAL_MDMA_LinkedList_CreateNode(mdma_ch1_node[i], &nodeConfig);
        if (ret_hal != HAL_OK)
        {
            Error_Handler();
        }
        ret_hal = HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel1_sw_0, mdma_ch1_node[i], 0);
        if (ret_hal != HAL_OK)
        {
            Error_Handler();
        }
    }

    HAL_MDMA_RegisterCallback(&hmdma_mdma_channel0_sw_0, HAL_MDMA_XFER_CPLT_CB_ID, mic_data_interlace_complete);
    HAL_MDMA_RegisterCallback(&hmdma_mdma_channel1_sw_0, HAL_MDMA_XFER_CPLT_CB_ID, mic_data_interlace_complete);
}

void register_mic_interlaced_data_ready_callback(MicInterlacedDataReadyIsrCallback callback)
{
    mic_interlaced_data_ready_callback = callback;
}

void mic_start()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;
    DFSDM_Filter_HandleTypeDef *const dfsdm_filters[] = {
        &hdfsdm1_filter0,
        &hdfsdm1_filter1,
        &hdfsdm1_filter2,
        &hdfsdm1_filter3,
    };

    for (size_t i = 0; i < sizeof(dfsdm_filters) / sizeof(dfsdm_filters[0]); i++)
    {
        ret_hal = HAL_DFSDM_FilterRegularMsbStart_DMA(dfsdm_filters[i], mic_data[i][0], DFSDM_DMA_FRAME_SAMPLE_NUM * 2);
        if (ret_hal != HAL_OK)
        {
            printf("hdfsdm1 filter%u start fail, code: %u", i, ret_hal);
            Error_Handler();
        }
    }
}

void mic_stop()
{
    HAL_StatusTypeDef ret_hal = HAL_OK;
    DFSDM_Filter_HandleTypeDef *const dfsdm_filters[] = {
        &hdfsdm1_filter0,
        &hdfsdm1_filter1,
        &hdfsdm1_filter2,
        &hdfsdm1_filter3,
    };

    for (size_t i = 0; i < sizeof(dfsdm_filters) / sizeof(dfsdm_filters[0]); i++)
    {
        ret_hal = HAL_DFSDM_FilterRegularStop_DMA(dfsdm_filters[i]);
        if (ret_hal != HAL_OK)
        {
            printf("hdfsdm1 filter%u stop fail, code: %u", i, ret_hal);
            Error_Handler();
        }
    }
}

bool mic_verify_interlaced_data()
{
    for (size_t i = 0; i < DFSDM_DMA_FRAME_SAMPLE_NUM; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            if (mic_data[j][mic_data_interlaced_from][i] != mic_data_interlaced[i * 4 + j])
            {
                return false;
            }
        }
    }
    return true;
}

static inline void dfsdm_dma_irq(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, const uint8_t idx)
{
    static uint32_t internal_flag = 0;
    const uint32_t INTERNAL_MIC1_FH_RDY_BIT = 1 << 0;
    const uint32_t INTERNAL_MIC2_FH_RDY_BIT = 1 << 1;
    const uint32_t INTERNAL_MIC3_FH_RDY_BIT = 1 << 2;
    const uint32_t INTERNAL_MIC4_FH_RDY_BIT = 1 << 3;
    const uint32_t INTERNAL_MIC1_SH_RDY_BIT = 1 << 4;
    const uint32_t INTERNAL_MIC2_SH_RDY_BIT = 1 << 5;
    const uint32_t INTERNAL_MIC3_SH_RDY_BIT = 1 << 6;
    const uint32_t INTERNAL_MIC4_SH_RDY_BIT = 1 << 7;
    const uint32_t INTERNAL_MIC_FH_RDY_BIT =
        INTERNAL_MIC1_FH_RDY_BIT | INTERNAL_MIC2_FH_RDY_BIT | INTERNAL_MIC3_FH_RDY_BIT | INTERNAL_MIC4_FH_RDY_BIT;
    const uint32_t INTERNAL_MIC_SH_RDY_BIT =
        INTERNAL_MIC1_SH_RDY_BIT | INTERNAL_MIC2_SH_RDY_BIT | INTERNAL_MIC3_SH_RDY_BIT | INTERNAL_MIC4_SH_RDY_BIT;

    if (hdfsdm_filter == &hdfsdm1_filter0)
    {
        ATOMIC_SET_BIT(internal_flag, idx == 0 ? INTERNAL_MIC1_FH_RDY_BIT : INTERNAL_MIC1_SH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter1)
    {
        ATOMIC_SET_BIT(internal_flag, idx == 0 ? INTERNAL_MIC2_FH_RDY_BIT : INTERNAL_MIC2_SH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter2)
    {
        ATOMIC_SET_BIT(internal_flag, idx == 0 ? INTERNAL_MIC3_FH_RDY_BIT : INTERNAL_MIC3_SH_RDY_BIT);
    }
    else if (hdfsdm_filter == &hdfsdm1_filter3)
    {
        ATOMIC_SET_BIT(internal_flag, idx == 0 ? INTERNAL_MIC4_FH_RDY_BIT : INTERNAL_MIC4_SH_RDY_BIT);
    }

    if ((internal_flag & INTERNAL_MIC_FH_RDY_BIT) == INTERNAL_MIC_FH_RDY_BIT)
    {
        ATOMIC_CLEAR_BIT(internal_flag, INTERNAL_MIC_FH_RDY_BIT);
        HAL_StatusTypeDef ret_hal = HAL_MDMA_Start_IT(&hmdma_mdma_channel0_sw_0, (uint32_t)&mic_data[0][0][0],
                                                      (uint32_t)&mic_data_interlaced[0], 2, DFSDM_DMA_FRAME_SAMPLE_NUM);
        if (ret_hal != HAL_OK)
        {
            Error_Handler();
        }
    }
    else if ((internal_flag & INTERNAL_MIC_SH_RDY_BIT) == INTERNAL_MIC_SH_RDY_BIT)
    {
        ATOMIC_CLEAR_BIT(internal_flag, INTERNAL_MIC_SH_RDY_BIT);
        HAL_StatusTypeDef ret_hal = HAL_MDMA_Start_IT(&hmdma_mdma_channel1_sw_0, (uint32_t)&mic_data[0][1][0],
                                                      (uint32_t)&mic_data_interlaced[0], 2, DFSDM_DMA_FRAME_SAMPLE_NUM);
        if (ret_hal != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    dfsdm_dma_irq(hdfsdm_filter, 0);
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    dfsdm_dma_irq(hdfsdm_filter, 1);
}
