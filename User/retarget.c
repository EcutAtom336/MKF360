#include "User/retarget.h"

#include <stdbool.h>

#include "lwrb/lwrb.h"
#include "stm32h7xx_hal.h"

#include "usart.h"

#define BUFFER_SIZE (4096U)
#define TRY_OUTPUT_THRESHOLD ((uint32_t)(BUFFER_SIZE * 0.8F))
#define MINIMAL_OUT_TICK_INTERVAL (50U)

__attribute__((section(".bss.DTCM"))) lwrb_t stdout_rb;
__attribute__((section(".bss.DMA_RAM_D2"))) uint8_t stdout_rb_buffer[BUFFER_SIZE];
__attribute__((section(".bss.DTCM"))) uint32_t last_out_tick;
__attribute__((section(".bss.DTCM"))) volatile bool tx_busy;
__attribute__((section(".bss.DTCM"))) uint32_t pendding_send_size;

inline void stdout_maintain()
{
    // 如果有正在进行的传输，先检测传输状态
    if (tx_busy)
    {
        return;
    }
    if (pendding_send_size != 0)
    {
        // 传输完成，释放内存
        lwrb_skip(&stdout_rb, pendding_send_size);
        pendding_send_size = 0;
    }

    const uint32_t current_tick = HAL_GetTick();
    const uint32_t full = lwrb_get_full(&stdout_rb);
    if (full == 0 || (current_tick - last_out_tick < MINIMAL_OUT_TICK_INTERVAL && full < TRY_OUTPUT_THRESHOLD))
    {
        return;
    }

    const uint32_t linear_length = lwrb_get_linear_block_read_length(&stdout_rb);
    const void *linear_address = lwrb_get_linear_block_read_address(&stdout_rb);

    HAL_StatusTypeDef ret_hal = HAL_UART_Transmit_DMA(&huart1, linear_address, linear_length);
    if (ret_hal == HAL_OK)
    {
        last_out_tick = HAL_GetTick();
        pendding_send_size = linear_length;
        tx_busy = true;
    }
    else
    {
        Error_Handler();
    }
}

void stdout_init()
{
    lwrb_init(&stdout_rb, &stdout_rb_buffer[0], sizeof(stdout_rb_buffer));
}

int stdout_putchar(int ch)
{
    const uint8_t uint8_ch = (uint8_t)ch;

    if (lwrb_get_free(&stdout_rb) != 0)
    {
        lwrb_write(&stdout_rb, &uint8_ch, 1);
    }

    stdout_maintain();

    return ch;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        tx_busy = false;
        huart->gState = HAL_UART_STATE_READY;
    }
}
