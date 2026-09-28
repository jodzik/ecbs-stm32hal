#include "ecbs_stm32hal.h"

#include <safe_c.h>

#include <string.h>

#include "stm32f1xx_hal.h"

// Size of the circular RX DMA ring buffer, bytes. Must exceed the worst-case
// backlog between ecbs_stm32hal__process() calls: at 9600 baud one byte
// arrives per ~1.04 ms, so 512 bytes cover ~530 ms main loop stall.
#ifndef CONFIG_ECBS_STM32HAL_RX_RING_SIZE
#define CONFIG_ECBS_STM32HAL_RX_RING_SIZE 512
#endif

#if (CONFIG_ECBS_STM32HAL_RX_RING_SIZE > 65535)
#error "CONFIG_ECBS_STM32HAL_RX_RING_SIZE exceeds the DMA counter range."
#endif

// The rx data path: the DMA writes the ring, the main loop drains it by
// ecbs_stm32hal__process(), the fill level is derived from the DMA counter.
// The tx data path: write() starts the DMA transfer directly on the caller
// buffer, the completion is reported by the TxCplt callback through
// get_write_status().
typedef enum EcbsHalTxState {
    ECBS_HAL_TX__IDLE = 0,
    ECBS_HAL_TX__BUSY,
    ECBS_HAL_TX__DONE,
    ECBS_HAL_TX__FAIL,
} EcbsHalTxState;

static struct Ecbs s_ecbs;
static UART_HandleTypeDef* s_huart = NULL;
static uint8_t s_rx_ring[CONFIG_ECBS_STM32HAL_RX_RING_SIZE];
static uint16_t s_rx_tail; // next position to read, owned by the main loop
static volatile EcbsHalTxState s_tx_state = ECBS_HAL_TX__IDLE;
static volatile uint8_t s_rx_rearm = 0; // set by the UART error, served by process()
static uint64_t s_time_base_ms;
static uint32_t s_time_last_tick;
static EcbsHalStatus s_status;

static uint16_t rx_head(void) {
    return (uint16_t)(sizeof(s_rx_ring) - (uint16_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx));
}

static uint64_t get_time_ms_cb(void) {
    uint32_t const now = HAL_GetTick();

    s_time_base_ms += (uint32_t)(now - s_time_last_tick);
    s_time_last_tick = now;

    return s_time_base_ms;
}

// The core drains the ring completely on each poll, so a partial overwrite
// of the unread data is not possible without a full ring overwrite, which
// the DMA counter cannot show. After such an overrun the byte stream is
// corrupted: the framer resyncs on the next frame, the CRC check drops the
// damaged packet and the master retries the request.
static int read_cb(uint8_t* const buf, uint16_t const buf_size) {
    int rc = 0;

    ASSERT(NULL != buf, ER_INVAL);
    if (0 == buf_size) {
        goto finally;
    }

    uint16_t const head = rx_head();
    uint16_t const tail = s_rx_tail;
    uint16_t const avail = (uint16_t)(head - tail);

    if (0 == avail) {
        goto finally;
    }

    uint16_t const n = (avail < buf_size) ? avail : buf_size;
    uint16_t const first = (uint16_t)(sizeof(s_rx_ring) - tail);

    if (first >= n) {
        memcpy(buf, &s_rx_ring[tail], n);
    } else {
        memcpy(buf, &s_rx_ring[tail], first);
        memcpy(&buf[first], &s_rx_ring[0], (uint16_t)(n - first));
    }
    s_rx_tail = (uint16_t)((tail + n) % sizeof(s_rx_ring));

    rc = n;

 finally:

    return rc;
}

static int write_cb(uint8_t const* const data, uint16_t const ndata) {
    int rc = 0;

    ASSERT((NULL != data) && (0 < ndata), ER_INVAL);

    if (ECBS_HAL_TX__BUSY == s_tx_state) {
        // The core retries with the rest of the frame on the next process call.
        rc = 0;
        goto finally;
    }

    // Set BUSY before the start: the transfer complete callback can fire
    // as soon as the DMA is armed inside HAL_UART_Transmit_DMA().
    s_tx_state = ECBS_HAL_TX__BUSY;
    HAL_StatusTypeDef const hal_rc = HAL_UART_Transmit_DMA(s_huart, data, ndata);
    if (HAL_OK == hal_rc) {
        rc = ndata;
        goto finally;
    }

    if (HAL_BUSY == hal_rc) {
        // The UART is busy outside of the library, retry later.
        s_tx_state = ECBS_HAL_TX__IDLE;
        rc = 0;
        goto finally;
    }

    s_tx_state = ECBS_HAL_TX__FAIL;
    s_status.tx_error_count++;
    rc = ER_IO;

 finally:

    return rc;
}

static enum EcbsWriteStatus get_write_status_cb(void) {
    switch (s_tx_state) {
    case ECBS_HAL_TX__BUSY:
        return ECBS_WRITE_STATUS__PROCEEDED;
    case ECBS_HAL_TX__DONE:
        s_tx_state = ECBS_HAL_TX__IDLE;
        return ECBS_WRITE_STATUS__COMPLETED;
    case ECBS_HAL_TX__FAIL:
        s_tx_state = ECBS_HAL_TX__IDLE;
        return ECBS_WRITE_STATUS__FAILED;
    default:
        break;
    }

    // Cannot be polled without a write, fail safe: the master will retry.
    return ECBS_WRITE_STATUS__FAILED;
}

static int rx_rearm(void) {
    int rc = 0;

    if (0 == s_rx_rearm) {
        goto finally;
    }
    s_rx_rearm = 0U;

    s_status.rx_restart_count++;

    if (HAL_UART_STATE_BUSY_RX == s_huart->RxState) {
        // The reception survived the error, only the diagnostics is updated.
        goto finally;
    }

    if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_rx_ring, (uint16_t)sizeof(s_rx_ring))) {
        LOG_ERR("Fail to restart the UART DMA reception");
        rc = ER_IO;
        goto finally;
    }

    s_rx_tail = 0;

 finally:

    return rc;
}

int ecbs_stm32hal__init(UART_HandleTypeDef* const huart, EcbsAddr const addr) {
    int rc = 0;

    ASSERTf((NULL != huart->hdmarx) && (NULL != huart->hdmatx), ER_NO_DEV,
        "UART DMA handles are not linked, configure them by CubeMX: huart=%p", (void*)huart);
    ASSERTf(ECBS__BROADCAST_ADDR != addr, ER_INVAL, "Broadcast address cannot be used: addr=%u", addr);

    memset(&s_status, 0, sizeof(s_status));
    s_huart = huart;
    s_rx_tail = 0;
    s_tx_state = ECBS_HAL_TX__IDLE;
    s_rx_rearm = 0U;
    s_time_base_ms = 0;
    s_time_last_tick = HAL_GetTick();

    TRY(ecbs__init(&s_ecbs, addr, get_time_ms_cb, read_cb, write_cb, get_write_status_cb));

    if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_ring, (uint16_t)sizeof(s_rx_ring))) {
        LOG_ERR("Fail to start the UART DMA reception");
        rc = ER_IO;
        goto finally;
    }

    s_status.initialized = 1U;

 finally:

    return rc;
}

struct Ecbs* ecbs_stm32hal__ecbs(void) {
    return (0 != s_status.initialized) ? &s_ecbs : NULL;
}

int ecbs_stm32hal__process(void) {
    if (0 == s_status.initialized) {
        return ER_NO_DEV;
    }

    int rc = rx_rearm();
    if (0 != rc) {
        s_rx_rearm = 1U; // Retry on the next process call.
        return rc;
    }

    return ecbs__process(&s_ecbs);
}

void ecbs_stm32hal__on_tx_complete(UART_HandleTypeDef* const huart) {
    if ((NULL == huart) || (huart != s_huart)) {
        return;
    }

    s_tx_state = ECBS_HAL_TX__DONE;
}

void ecbs_stm32hal__on_rx_event(UART_HandleTypeDef* const huart, uint16_t const size) {
    (void)size;

    if ((NULL == huart) || (huart != s_huart)) {
        return;
    }

    s_status.rx_event_count++;
}

void ecbs_stm32hal__on_uart_error(UART_HandleTypeDef* const huart) {
    if ((NULL == huart) || (huart != s_huart)) {
        return;
    }

    s_status.uart_error_count++;
    s_rx_rearm = 1U;

    if ((ECBS_HAL_TX__BUSY == s_tx_state) && (HAL_UART_STATE_BUSY_TX != huart->gState)) {
        // The transfer is aborted without the complete callback. If it still
        // completes later, the callback corrects the state.
        s_tx_state = ECBS_HAL_TX__FAIL;
        s_status.tx_error_count++;
    }
}

uint16_t ecbs_stm32hal__max_payload_size(void) {
    return ECBS__MAX_PAYLOAD_SIZE;
}

void ecbs_stm32hal__get_status(EcbsHalStatus* const status) {
    *status = s_status;
}
