#include <ecbs_stm32hal.h>
#include <ecbs.h>
#include <framer7b.h>
#include <crc32.h>
#include <byteorder.h>
#include <safe_c.h>

#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define CHECK(cond)                                                                     \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            fprintf(stderr, "  CHECK FAILED %s:%i: %s\n", __func__, __LINE__, #cond);   \
            return -1;                                                                  \
        }                                                                               \
    } while (0)

enum {
    TEST__SLAVE_ADDR = 1,
    TEST__TID = 1,
    TEST__UNKNOWN_DATA_ID = 555,
    TEST__RESET_DATA_ID = 65280,
    TEST__TX_BUF_SIZE = 4096,

    TEST__PD_POS = 0,
    TEST__ADDR_POS = 1,
    TEST__TID_POS = 2,
    TEST__DATA_ID_POS = 4,
    TEST__PAYLOAD_POS = 6,
    TEST__CRC_SIZE = 4,

    TEST__PD_DIR_REQ = 0x80,
    TEST__PD_TYPE_MASK = 0x0F,
};

// Stub HAL state, driven by the test.
static UART_HandleTypeDef g_huart;
static DMA_HandleTypeDef g_dma_tx;
static DMA_HandleTypeDef g_dma_rx;
static DMA_TypeDef g_dma_rx_regs;

static uint8_t* g_ring = NULL;
static uint16_t g_ring_size = 0;
static uint16_t g_ring_head = 0;

static uint8_t g_tx_buf[TEST__TX_BUF_SIZE];
static uint16_t g_tx_size = 0;
static int g_tx_start_count = 0;
static HAL_StatusTypeDef g_tx_next_rc = HAL_OK;

static uint32_t g_tick_ms = 0;
static int g_rx_start_count = 0;
static HAL_StatusTypeDef g_rx_next_rc = HAL_OK;

static void safe_c_print(char const* const str) {
    (void)fputs(str, stderr);
}

uint32_t HAL_GetTick(void) {
    return g_tick_ms;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* const huart, const uint8_t* const pData, uint16_t const Size) {
    if ((NULL == huart) || (huart != &g_huart) || (NULL == pData)) {
        return HAL_ERROR;
    }
    if (HAL_OK != g_tx_next_rc) {
        return g_tx_next_rc;
    }

    memcpy(g_tx_buf, pData, Size);
    g_tx_size = Size;
    g_tx_start_count++;
    g_huart.gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* const huart, uint8_t* const pData, uint16_t const Size) {
    if ((NULL == huart) || (huart != &g_huart) || (NULL == pData)) {
        return HAL_ERROR;
    }
    if (HAL_OK != g_rx_next_rc) {
        return g_rx_next_rc;
    }

    g_ring = pData;
    g_ring_size = Size;
    g_ring_head = 0;
    g_dma_rx_regs.CNDTR = Size;
    g_huart.RxState = HAL_UART_STATE_BUSY_RX;
    g_rx_start_count++;
    return HAL_OK;
}

// Simulate the DMA hardware: write bytes into the ring and advance the head.
static void dma_rx_write(uint8_t const* const data, uint16_t const n) {
    for (uint16_t i = 0; i < n; i++) {
        g_ring[g_ring_head] = data[i];
        g_ring_head = (uint16_t)((g_ring_head + 1U) % g_ring_size);
        g_dma_rx_regs.CNDTR = (uint32_t)(g_ring_size - g_ring_head);
    }
}

static void complete_tx(void) {
    g_huart.gState = HAL_UART_STATE_READY;
    ecbs_stm32hal__on_tx_complete(&g_huart);
}

static uint16_t build_request(uint8_t* const buf, uint16_t const buf_size, uint8_t const pd, uint8_t const addr,
    uint16_t const tid, uint16_t const data_id, uint8_t const* const payload, uint16_t const payload_size) {
    buf[TEST__PD_POS] = pd;
    buf[TEST__ADDR_POS] = addr;
    u16_to_le(&buf[TEST__TID_POS], tid);
    u16_to_le(&buf[TEST__DATA_ID_POS], data_id);
    if ((NULL != payload) && (0 < payload_size)) {
        memcpy(&buf[TEST__PAYLOAD_POS], payload, payload_size);
    }

    uint16_t const body_size = (uint16_t)(TEST__PAYLOAD_POS + payload_size);
    u32_to_le(&buf[body_size], crc32__ieee(buf, body_size));

    int const frame_size = framer7b__encode_in_place(buf, (uint16_t)(body_size + TEST__CRC_SIZE), buf_size);
    return (0 < frame_size) ? (uint16_t)frame_size : 0;
}

// Decode the captured transport answer into the plain packet buffer.
static uint16_t parse_answer(uint8_t* const buf, uint16_t const buf_size) {
    Framer7bReceiver framer = {0};

    CHECK(0 == framer7b_receiver__init(&framer, buf, buf_size));
    for (uint16_t i = 0; i < g_tx_size; i++) {
        int const rc = framer7b_receiver__push(&framer, g_tx_buf[i]);
        if (0 < rc) {
            return (uint16_t)rc;
        }
        CHECK(0 == rc);
    }

    return 0;
}

static void reset_stubs(void) {
    memset(&g_huart, 0, sizeof(g_huart));
    g_huart.hdmatx = &g_dma_tx;
    g_huart.hdmarx = &g_dma_rx;
    g_dma_rx.Instance = &g_dma_rx_regs;

    g_ring = NULL;
    g_ring_size = 0;
    g_ring_head = 0;
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    g_tx_size = 0;
    g_tx_start_count = 0;
    g_tx_next_rc = HAL_OK;
    g_tick_ms = 0;
    g_rx_start_count = 0;
    g_rx_next_rc = HAL_OK;
}

static int test_init_guards(void) {
    reset_stubs();

    g_huart.hdmarx = NULL;
    CHECK(ER_NO_DEV == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));
    g_huart.hdmarx = &g_dma_rx;
    g_huart.hdmatx = NULL;
    CHECK(ER_NO_DEV == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));
    g_huart.hdmatx = &g_dma_tx;

    CHECK(ER_INVAL == ecbs_stm32hal__init(&g_huart, ECBS__BROADCAST_ADDR));

    return 0;
}

static int test_app_err_answer(void) {
    uint8_t frame[512] = {0};
    uint8_t packet[512] = {0};

    reset_stubs();
    CHECK(0 == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));
    CHECK(NULL != ecbs_stm32hal__ecbs());
    CHECK(CONFIG_ECBS_MAX_PAYLOAD_SIZE == ecbs_stm32hal__max_payload_size());

    // WRITE to an unknown data_id -> APP_ERR answer with NO_DATA_ID.
    uint8_t const payload[] = "hi";
    uint16_t const size = build_request(frame, sizeof(frame), TEST__PD_DIR_REQ | 0x00, TEST__SLAVE_ADDR,
        TEST__TID, TEST__UNKNOWN_DATA_ID, payload, sizeof(payload));
    CHECK(0 < size);
    dma_rx_write(frame, size);

    CHECK(0 == ecbs_stm32hal__process()); // the request is handled
    CHECK(0 == ecbs_stm32hal__process()); // the transport write starts
    CHECK(1 == g_tx_start_count);

    // Without the completion the write is still in progress.
    CHECK(0 == ecbs_stm32hal__process());
    CHECK(1 == g_tx_start_count);
    complete_tx();
    CHECK(0 == ecbs_stm32hal__process()); // the completion is polled

    uint16_t const answer_size = parse_answer(packet, sizeof(packet));
    CHECK(answer_size == (TEST__PAYLOAD_POS + 1 + TEST__CRC_SIZE));
    CHECK(0x0E == (packet[TEST__PD_POS] & TEST__PD_TYPE_MASK));
    CHECK(0 == (packet[TEST__PD_POS] & TEST__PD_DIR_REQ));
    CHECK(TEST__SLAVE_ADDR == packet[TEST__ADDR_POS]);
    CHECK(TEST__TID == u16_from_le(&packet[TEST__TID_POS]));
    CHECK(TEST__UNKNOWN_DATA_ID == u16_from_le(&packet[TEST__DATA_ID_POS]));
    CHECK(ECBS_APP_ERR__NO_DATA_ID == packet[TEST__PAYLOAD_POS]);
    CHECK(u32_from_le(&packet[answer_size - TEST__CRC_SIZE]) == crc32__ieee(packet, answer_size - TEST__CRC_SIZE));

    return 0;
}

static int test_retry_dedup(void) {
    uint8_t frame[512] = {0};

    reset_stubs();
    CHECK(0 == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));

    uint8_t const payload[] = "hi";
    uint16_t const size = build_request(frame, sizeof(frame), TEST__PD_DIR_REQ | 0x00, TEST__SLAVE_ADDR,
        TEST__TID, TEST__UNKNOWN_DATA_ID, payload, sizeof(payload));
    CHECK(0 < size);

    // The same request twice: the second one is answered by the stored answer.
    dma_rx_write(frame, size);
    CHECK(0 == ecbs_stm32hal__process());
    CHECK(0 == ecbs_stm32hal__process());
    CHECK(1 == g_tx_start_count);
    complete_tx();
    CHECK(0 == ecbs_stm32hal__process());

    dma_rx_write(frame, size);
    CHECK(0 == ecbs_stm32hal__process());
    CHECK(0 == ecbs_stm32hal__process());
    CHECK(2 == g_tx_start_count);
    complete_tx();
    CHECK(0 == ecbs_stm32hal__process());

    return 0;
}

static int test_tx_busy_retry(void) {
    uint8_t frame[512] = {0};

    reset_stubs();
    CHECK(0 == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));

    uint8_t const payload[] = "hi";
    uint16_t const size = build_request(frame, sizeof(frame), TEST__PD_DIR_REQ | 0x00, TEST__SLAVE_ADDR,
        TEST__TID, TEST__UNKNOWN_DATA_ID, payload, sizeof(payload));
    CHECK(0 < size);

    // The transport cannot accept the answer now, the core retries later.
    dma_rx_write(frame, size);
    g_tx_next_rc = HAL_BUSY;
    CHECK(0 == ecbs_stm32hal__process()); // the request is handled
    CHECK(0 == g_tx_start_count);
    CHECK(0 == ecbs_stm32hal__process()); // the write is refused, the frame is kept
    CHECK(0 == g_tx_start_count);

    g_tx_next_rc = HAL_OK;
    CHECK(0 == ecbs_stm32hal__process()); // the retry starts the write
    CHECK(1 == g_tx_start_count);
    complete_tx();
    CHECK(0 == ecbs_stm32hal__process());

    return 0;
}

static int test_rx_noise_resync(void) {
    uint8_t frame[512] = {0};
    uint8_t noise[512] = {0};

    reset_stubs();
    CHECK(0 == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));

    uint8_t const payload[] = "hi";
    uint16_t const size = build_request(frame, sizeof(frame), TEST__PD_DIR_REQ | 0x00, TEST__SLAVE_ADDR,
        TEST__TID, TEST__UNKNOWN_DATA_ID, payload, sizeof(payload));
    CHECK(0 < size);

    // Garbage before a valid frame(the ring overrun aftermath): the framer
    // resyncs on the frame start and the answer is still produced.
    for (uint16_t i = 0; i < sizeof(noise); i++) {
        noise[i] = (uint8_t)(0x41U + (i % 26U));
    }
    dma_rx_write(noise, sizeof(noise));
    dma_rx_write(frame, size);

    CHECK(0 == ecbs_stm32hal__process()); // the noise is dropped
    CHECK(0 == ecbs_stm32hal__process()); // the answer write starts
    CHECK(1 == g_tx_start_count);
    complete_tx();
    CHECK(0 == ecbs_stm32hal__process());

    return 0;
}

static int test_rx_error_rearm(void) {
    EcbsHalStatus status = {0};

    reset_stubs();
    CHECK(0 == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));
    CHECK(1 == g_rx_start_count);

    // The reception survived the error: only the counter grows.
    ecbs_stm32hal__on_uart_error(&g_huart);
    CHECK(0 == ecbs_stm32hal__process());
    ecbs_stm32hal__get_status(&status);
    CHECK(1 == status.uart_error_count);
    CHECK(1 == status.rx_restart_count);
    CHECK(1 == g_rx_start_count);

    // The reception stopped: it is re-armed by process().
    g_huart.RxState = HAL_UART_STATE_READY;
    ecbs_stm32hal__on_uart_error(&g_huart);
    CHECK(0 == ecbs_stm32hal__process());
    ecbs_stm32hal__get_status(&status);
    CHECK(2 == status.uart_error_count);
    CHECK(2 == status.rx_restart_count);
    CHECK(2 == g_rx_start_count);
    CHECK(HAL_UART_STATE_BUSY_RX == g_huart.RxState);

    return 0;
}

static int test_announce(void) {
    uint8_t packet[512] = {0};

    reset_stubs();
    CHECK(0 == ecbs_stm32hal__init(&g_huart, TEST__SLAVE_ADDR));

    CHECK(0 == ecbs__announce(ecbs_stm32hal__ecbs()));
    CHECK(0 == ecbs_stm32hal__process()); // the transport write starts
    CHECK(1 == g_tx_start_count);
    complete_tx();
    CHECK(0 == ecbs_stm32hal__process());

    uint16_t const answer_size = parse_answer(packet, sizeof(packet));
    CHECK(answer_size == (TEST__PAYLOAD_POS + TEST__CRC_SIZE));
    CHECK(0x00 == (packet[TEST__PD_POS] & TEST__PD_TYPE_MASK));
    CHECK(0 == (packet[TEST__PD_POS] & TEST__PD_DIR_REQ));
    CHECK(TEST__SLAVE_ADDR == packet[TEST__ADDR_POS]);
    CHECK(0 == u16_from_le(&packet[TEST__TID_POS]));
    CHECK(TEST__RESET_DATA_ID == u16_from_le(&packet[TEST__DATA_ID_POS]));

    return 0;
}

static int test_not_initialized(void) {
    reset_stubs();

    CHECK(NULL == ecbs_stm32hal__ecbs());
    CHECK(ER_NO_DEV == ecbs_stm32hal__process());
    // The callbacks of foreign or not initialized UARTs must be ignored.
    ecbs_stm32hal__on_tx_complete(&g_huart);
    ecbs_stm32hal__on_rx_event(&g_huart, 0);
    ecbs_stm32hal__on_uart_error(&g_huart);

    return 0;
}

int main(void) {
    int rc = 0;
    int check = 0;

    safe_c__init(safe_c_print);

    // Must run first: the adapter state is static and persists between tests.
    check = test_not_initialized();
    if (0 != check) rc = check;
    check = test_init_guards();
    if (0 != check) rc = check;
    check = test_app_err_answer();
    if (0 != check) rc = check;
    check = test_retry_dedup();
    if (0 != check) rc = check;
    check = test_tx_busy_retry();
    if (0 != check) rc = check;
    check = test_rx_noise_resync();
    if (0 != check) rc = check;
    check = test_rx_error_rearm();
    if (0 != check) rc = check;
    check = test_announce();
    if (0 != check) rc = check;

    if (0 == rc) {
        printf("ecbs_stm32hal TEST OK\n");
    } else {
        printf("ecbs_stm32hal TEST FAILED\n");
    }

    return (0 == rc) ? 0 : 1;
}
