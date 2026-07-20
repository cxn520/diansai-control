#include "wit.h"

uint8_t wit_dmaBuffer[33];

/* UART中断与控制任务共享，volatile保证每次控制计算都重新读取最新IMU数据。 */
volatile WIT_Data_t wit_data;

void WIT_Init(void)
{
    DL_DMA_setSrcAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t)(&UART_WIT_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);

    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
}
