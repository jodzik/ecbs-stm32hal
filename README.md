# ECB Slave STM32 HAL

STM32 HAL transport adapter for the [ecbs-c](https://github.com/jodzik/ecbs-c)
ECB slave core: circular DMA reception, zero-copy DMA transmission.

## Design

- RX: `HAL_UARTEx_ReceiveToIdle_DMA()` into an internal circular ring, armed
  once at init and never stopped in the normal flow. The `read()` callback of
  the core derives the fill level from the DMA counter, so the interrupt
  context only serves as a wake-up source. After a ring overrun(too long main
  loop stall) the byte stream is corrupted, this is not detectable at the
  transport level: the framer resyncs on the next frame, the CRC check drops
  the damaged packet and the master retries the request.
- TX: `HAL_UART_Transmit_DMA()` started directly on the `tx_buf` slice offered
  by the core(zero copy). The optional `get_write_status()` transport callback
  of the core maps the completion reported by `HAL_UART_TxCpltCallback()`.
  Completion means the DMA transfer complete: the last byte can still be
  ~one byte time on the wire, which is irrelevant against the master timeout.
- Time: wrap-safe 64-bit extension of `HAL_GetTick()`.
- Concurrency: the single-writer(DMA)/single-reader(main loop) ring needs no
  locking, the ISRs only set flags/counters, everything else runs in the
  caller context of `ecbs_stm32hal__process()`, as required by the core.

## Integration

The UART and the DMA channels are configured by the application(CubeMX):

- USARTx: as needed, interrupt enabled(the IDLE events wake `__WFI()`),
- USARTx_RX: DMA channel, circular mode, byte alignment, `hdmarx` linked,
- USARTx_TX: DMA channel, normal mode, byte alignment, `hdmatx` linked,
- both DMA channel interrupts enabled, `MX_DMA_Init()` called before the
  UART init(DMA clock and NVIC).

```c
#include "stm32f1xx_hal.h"
#include <ecbs_stm32hal.h>

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    ecbs_stm32hal__on_tx_complete(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    ecbs_stm32hal__on_rx_event(huart, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    ecbs_stm32hal__on_uart_error(huart);
}

int main(void) {
    ...
    MX_USART1_UART_Init();

    ecbs_stm32hal__init(&huart1, 1);
    ecbs__announce(ecbs_stm32hal__ecbs());
    ecbs__register_endpoint(ecbs_stm32hal__ecbs(), ...);

    while (1) {
        ecbs_stm32hal__process();
    }
}
```

The library owns the `struct Ecbs` instance: register the endpoints and
publish data by `ecbs__send_async()` through `ecbs_stm32hal__ecbs()`.

## Configuration

Build options of every translation unit that includes `ecbs.h`
(the buffer layouts must match, cross-check at runtime by
`ecbs_stm32hal__max_payload_size()`):

- `CONFIG_ECBS_MAX_PAYLOAD_SIZE` - see ecbs-c, required,
- `CONFIG_ECBS_MAX_ENDPOINTS` - see ecbs-c, required.

Adapter local:

- `CONFIG_ECBS_STM32HAL_RX_RING_SIZE` - circular RX ring size, default 512.
  Must exceed the worst-case backlog between `ecbs_stm32hal__process()` calls:
  at 9600 baud one byte arrives per ~1.04 ms, so the default covers ~530 ms.

## Diagnostics

`ecbs_stm32hal__get_status()` returns the error/event counters, see
`EcbsHalStatus`. `rx_restart_count` grows on every UART error(a restart is
only attempted when the reception really stopped).

## Tests

`TEST/test.sh` builds and runs the host side test with a stubbed HAL
(see `TEST/stub/`), it needs the sibling library checkouts(`../../ecbs-c`,
`../../framer7b-c`, `../../safe-c`, `../../crc-c`, `../../byteorder-c`).
