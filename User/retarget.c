
#include "usart.h"

#include "stm32h7xx_hal.h"

#define BUFFER_SIZE (512)
#define LINEAR_SIZE(start, end, all) (start <= end ? end - start : all - start)
#define FULL_SIZE(start, end, all) (start <= end ? end - start : all - start + end)

__attribute__((section(".DMA_RAM_D2"))) uint8_t stdout_buffer[BUFFER_SIZE];
__attribute__((section(".DTCM"))) uint16_t w = UINT16_C(0);
__attribute__((section(".DTCM"))) uint16_t r = UINT16_C(0);
__attribute__((section(".DTCM"))) uint16_t remain = UINT16_MAX;
__attribute__((section(".DTCM"))) uint16_t next_length = UINT16_MAX;

int stdout_putchar(int ch)
{
    const uint8_t uint8_ch = (uint8_t)ch;

    stdout_buffer[w++] = uint8_ch;
    if (w >= BUFFER_SIZE)
    {
        w = 0;
    }

    if (uint8_ch == '\n')
    {
        const uint16_t linear_read_size = LINEAR_SIZE(w, r, BUFFER_SIZE);
        const uint16_t all_size = FULL_SIZE(w, r, BUFFER_SIZE);

        HAL_StatusTypeDef ret_hal = HAL_UART_Transmit_DMA(&huart1, &stdout_buffer[r], linear_read_size);
        if (ret_hal != HAL_OK)
        {
            next_length = all_size;
        }
        else if (linear_read_size != all_size)
        {
            remain = all_size - linear_read_size;
        }
    }

    return ch;
}

void uart_irq_handler(UART_HandleTypeDef *huart)
{
    r += huart->TxXferSize;
    HAL_StatusTypeDef ret_hal = HAL_OK;
    if (r >= BUFFER_SIZE)
    {
        r = 0;
        if (remain != UINT16_MAX)
        {
            const uint16_t size = remain;
            remain = UINT16_MAX;
            ret_hal = HAL_UART_Transmit_DMA(&huart1, &stdout_buffer[r], size);
            while (ret_hal != HAL_OK)
                ;
            return;
        }
    }

    if (next_length != UINT16_MAX)
    {
        const uint16_t lf_pos = r + next_length < BUFFER_SIZE ? r + next_length : r + next_length - BUFFER_SIZE;
        next_length = UINT16_MAX;
        const uint16_t linear_read_size = LINEAR_SIZE(r, lf_pos, BUFFER_SIZE);
        const uint16_t all_size = FULL_SIZE(r, lf_pos, BUFFER_SIZE);

        HAL_StatusTypeDef ret_hal = HAL_UART_Transmit_DMA(&huart1, &stdout_buffer[r], linear_read_size);
        while (ret_hal != HAL_OK)
            ;
        if (linear_read_size != all_size)
        {
            remain = all_size - linear_read_size;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uart_irq_handler(huart);
    }
}
