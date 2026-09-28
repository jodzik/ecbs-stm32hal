// Minimal STM32 HAL stub for the host side test of ecbs_stm32hal.c.
// Only the API surface used by the adapter is provided, the signatures
// repeat stm32f1xx_hal_uart.h / stm32f1xx_hal_dma.h.
#ifndef STUB_STM32F1XX_HAL_H_
#define STUB_STM32F1XX_HAL_H_

#include <stdint.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U,
} HAL_StatusTypeDef;

typedef enum {
    HAL_UART_STATE_RESET = 0x00U,
    HAL_UART_STATE_READY = 0x20U,
    HAL_UART_STATE_BUSY = 0x21U,
    HAL_UART_STATE_BUSY_TX = 0x21U,
    HAL_UART_STATE_BUSY_RX = 0x22U,
    HAL_UART_STATE_BUSY_TX_RX = 0x23U,
    HAL_UART_STATE_TIMEOUT = 0xA0U,
    HAL_UART_STATE_ERROR = 0xE0U,
} HAL_UART_StateTypeDef;

typedef struct {
    volatile uint32_t CNDTR;
} DMA_TypeDef;

typedef struct DMA_HandleTypeDef {
    DMA_TypeDef* Instance;
} DMA_HandleTypeDef;

typedef struct __UART_HandleTypeDef {
    DMA_HandleTypeDef* hdmarx;
    DMA_HandleTypeDef* hdmatx;
    volatile HAL_UART_StateTypeDef gState;
    volatile HAL_UART_StateTypeDef RxState;
} UART_HandleTypeDef;

#define __HAL_DMA_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNDTR)

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);

#endif // STUB_STM32F1XX_HAL_H_
