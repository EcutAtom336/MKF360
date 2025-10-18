#include "User/retarget.h"

#include "usart.h"

#include "stm32h7xx_hal.h"

#define BUFFER_SIZE (1024U)
#define TRY_OUTPUT_THRESHOLD ((uint32_t)(BUFFER_SIZE * 0.8F))
#define MINIMAL_OUT_TICK_INTERVAL (50U)

__attribute__((section(".bss.DMA_RAM_D2"))) uint8_t stdout_buffer[2][BUFFER_SIZE];
__attribute__((section(".bss.DTCM"))) uint32_t idle_buffer_full_size;
__attribute__((section(".bss.DTCM"))) uint8_t current_write_buffer_idx;
__attribute__((section(".bss.DTCM"))) uint32_t last_out_tick;

static inline void switch_buffer()
{
    current_write_buffer_idx = current_write_buffer_idx == 0 ? 1 : 0;
    idle_buffer_full_size = 0U;
}

static inline void try_output()
{
    if (idle_buffer_full_size == 0U ||
        (idle_buffer_full_size < TRY_OUTPUT_THRESHOLD && HAL_GetTick() - last_out_tick < MINIMAL_OUT_TICK_INTERVAL))
    {
        return;
    }

    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY || HAL_DMA_GetState(huart1.hdmatx) != HAL_DMA_STATE_READY)
    {
        return;
    }

    HAL_StatusTypeDef ret_hal =
        HAL_UART_Transmit_DMA(&huart1, &stdout_buffer[current_write_buffer_idx][0], idle_buffer_full_size);
    if (ret_hal == HAL_OK)
    {
        last_out_tick = HAL_GetTick();
        switch_buffer();
    }
    else
    {
        Error_Handler();
    }
}

int stdout_putchar(int ch)
{
    const uint8_t uint8_ch = (uint8_t)ch;

    if (idle_buffer_full_size < BUFFER_SIZE)
    {
        stdout_buffer[current_write_buffer_idx][idle_buffer_full_size++] = uint8_ch;
    }

    try_output();

    return ch;
}

void flush_stdout()
{
    try_output();
}
