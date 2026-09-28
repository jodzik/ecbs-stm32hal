#ifndef _ECBS_STM32HAL_H_
#define _ECBS_STM32HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ecbs.h>

#include <stdint.h>

// The HAL is not included here to keep the header usable in translation units
// without it, the typedef repeats stm32f1xx_hal_uart.h identically.
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/** Transport diagnostics, see #ecbs_stm32hal__get_status(). */
typedef struct EcbsHalStatus {
    uint8_t initialized; // 1 after successful ecbs_stm32hal__init()
    uint32_t uart_error_count; // count of HAL_UART_ErrorCallback() calls
    uint32_t rx_event_count; // count of rx DMA/IDLE events, wake-ups only
    uint32_t rx_restart_count; // rx re-arms after an UART error
    uint32_t tx_error_count; // TX DMA start failures and aborts
} EcbsHalStatus;

/** @brief Init the ECB slave transport on the UART.
 *
 * The UART must be configured by the application(CubeMX): RX DMA channel in
 * circular mode linked to huart->hdmarx, TX DMA channel in normal mode linked
 * to huart->hdmatx, the USARTx and both DMA channel interrupts enabled.
 * Starts the circular DMA reception into the internal ring buffer.
 * The library does not touch the HAL callbacks, the application routes them,
 * see ecbs_stm32hal__on_*().
 *
 * @return 0 if success, else #ErrorCodes:
 *         #ER_NO_DEV - UART DMA handles are not linked,
 *         #ER_INVAL - broadcast address,
 *         #ER_IO - fail to start the reception.
 */
int ecbs_stm32hal__init(UART_HandleTypeDef* huart, EcbsAddr addr) __nonnull((1));

/** @brief Get the ECB slave core instance, NULL before #ecbs_stm32hal__init().
 *
 * Use it to register the endpoints and to publish asynchronous data
 * by ecbs__send_async() when the bus is free.
 */
struct Ecbs* ecbs_stm32hal__ecbs(void);

/** @brief Process the ECB slave, call it periodically from the main loop.
 *
 * Also re-arms the reception after an UART error.
 *
 * @return 0 or transport #ErrorCodes, see ecbs__process().
 */
int ecbs_stm32hal__process(void);

/** @brief Feed HAL_UART_TxCpltCallback(), completes the transport write. */
void ecbs_stm32hal__on_tx_complete(UART_HandleTypeDef* huart);

/** @brief Feed HAL_UARTEx_RxEventCallback(), used for diagnostics only. */
void ecbs_stm32hal__on_rx_event(UART_HandleTypeDef* huart, uint16_t size);

/** @brief Feed HAL_UART_ErrorCallback(), fails the aborted transport write,
 *          the reception is re-armed by the next #ecbs_stm32hal__process(). */
void ecbs_stm32hal__on_uart_error(UART_HandleTypeDef* huart);

/** @brief Get CONFIG_ECBS_MAX_PAYLOAD_SIZE of this translation unit,
 *          the application cross-checks it against its own configuration. */
uint16_t ecbs_stm32hal__max_payload_size(void);

/** @brief Get the transport diagnostics copy. */
void ecbs_stm32hal__get_status(EcbsHalStatus* status) __nonnull((1));

#ifdef __cplusplus
}
#endif

#endif // _ECBS_STM32HAL_H_
