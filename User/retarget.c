
#include "usart.h"

#include "stm32h7xx_hal.h"

#define BUFFER_SIZE (512)

__attribute__((section(".DMA_RAM_D2"))) uint8_t stdout_buffer[2][BUFFER_SIZE];
__attribute__((section(".DTCM"))) uint8_t full_size[2] = {UINT8_C(0)};
__attribute__((section(".DTCM"))) uint8_t current_write_buffer_idx = UINT8_C(0);

static inline void switch_buffer()
{
    current_write_buffer_idx = current_write_buffer_idx == 0 ? 1 : 0;
}

int stdout_putchar(int ch)
{
    const uint8_t uint8_ch = (uint8_t)ch;
    stdout_buffer[current_write_buffer_idx][full_size[current_write_buffer_idx]++] = uint8_ch;

    if (uint8_ch == '\n')
    {
        HAL_StatusTypeDef ret_hal = HAL_UART_Transmit_DMA(&huart1, stdout_buffer[current_write_buffer_idx],
                                                          full_size[current_write_buffer_idx]);
        if (ret_hal == HAL_OK)
        {
            switch_buffer();
        }
    }

    return uint8_ch;
}

static void uart_irq_handler()
{
    full_size[current_write_buffer_idx == 0 ? 1 : 0] = 0;

    if (full_size[current_write_buffer_idx] != 0)
    {
        switch_buffer();
        HAL_StatusTypeDef ret_hal = HAL_UART_Transmit_DMA(&huart1, stdout_buffer[current_write_buffer_idx == 0 ? 1 : 0],
                                                          full_size[current_write_buffer_idx == 0 ? 1 : 0]);
        while (ret_hal != HAL_OK)
            ;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uart_irq_handler();
    }
}
