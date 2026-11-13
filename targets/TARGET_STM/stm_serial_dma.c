/* mbed Microcontroller Library
 * Copyright (c) 2026 HOREICH GmbH
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* hal/serial_dma_api.h for the STM32 families with stm_dma_ll.h (L4, U5).
 *
 * The UART is set up by serial_api.c (pins, clock, HAL_UART_Init); everything that runs per transfer or per
 * interrupt is LL and register access - no HAL_UART_*, no HAL_DMA_* handle bookkeeping:
 *  - the DMA channels are claimed through stm_dma_utils (so SPI, QSPI ... and this never share one), then driven by
 *    stm_dma_ll.h; their interrupts come here through stm_set_dma_irq_hook();
 *  - the UART's interrupt vector is this file's while attached (NVIC_SetVector, as the asynchronous API does), one
 *    per UART index, so any number of UARTs can be attached;
 *  - HAL_UART_IRQHandler is never called: it would end a DMA reception at the first framing or noise error and take
 *    TCIE for itself.
 */

#include "hal/serial_dma_api.h"

#if DEVICE_SERIAL_DMA

#include "platform/mbed_critical.h"
#include "serial_api_hal.h"
#include "stm_dma_info.h"
#include "stm_dma_ll.h"

#include <string.h>

#if !STM_DMA_LL_SUPPORTED
#error "SERIAL_DMA needs stm_dma_ll.h for this family"
#endif

// Defined in serial_api.c
extern int8_t get_uart_index(UARTName uart_name);

#define SERIAL_DMA_LINKS (sizeof(UARTTxDMALinks) / sizeof(UARTTxDMALinks[0]))
#define SERIAL_DMA_MAX_UARTS 8

_Static_assert(SERIAL_DMA_LINKS <= SERIAL_DMA_MAX_UARTS, "more UARTs than serial_dma vectors");

// TXE is TXFNF on parts with a UART FIFO (U5, L4+), and named so by LL there alone
#if defined(USART_CR1_FIFOEN)
#define SERIAL_DMA_TX_EMPTY(uart) LL_USART_IsActiveFlag_TXE_TXFNF(uart)
#else
#define SERIAL_DMA_TX_EMPTY(uart) LL_USART_IsActiveFlag_TXE(uart)
#endif

/// Bounds the polled waits: a UART that stopped must not hang a fatal error for ever (at 9600 baud a byte is ~1 ms).
#define SERIAL_DMA_POLL_LIMIT 1000000u

#define SERIAL_DMA_LINE_ERRORS (USART_ISR_FE | USART_ISR_NE | USART_ISR_PE | USART_ISR_ORE)

static serial_dma_t *serial_dma_attached[SERIAL_DMA_MAX_UARTS];

// ---- Interrupts ------------------------------------------------------------------------------------------------------

static void serial_dma_uart_irq(serial_dma_t *obj)
{
    if (obj == NULL) {
        return;
    }
    USART_TypeDef *uart = obj->uart;
    const uint32_t isr = LL_USART_ReadReg(uart, ISR);
    const uint32_t cr1 = LL_USART_ReadReg(uart, CR1);
    uint32_t events = 0;
    if ((cr1 & USART_CR1_IDLEIE) && (isr & USART_ISR_IDLE)) {
        LL_USART_ClearFlag_IDLE(uart);
        events |= SERIAL_DMA_RX; // the line went quiet after a burst
    }
    if ((cr1 & USART_CR1_TCIE) && (isr & USART_ISR_TC)) {
        LL_USART_DisableIT_TC(uart);
        events |= SERIAL_DMA_TX_IDLE;
    }
    if (isr & SERIAL_DMA_LINE_ERRORS) {
        LL_USART_WriteReg(uart, ICR, USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_ORECF);
        events |= SERIAL_DMA_ERROR; // the DMA reads on
    }
    if (events != 0) {
        obj->handler(obj->context, events);
    }
}

#define SERIAL_DMA_VECTOR(n) \
    static void serial_dma_uart_irq_##n(void) \
    { \
        serial_dma_uart_irq(serial_dma_attached[n]); \
    }
SERIAL_DMA_VECTOR(0)
SERIAL_DMA_VECTOR(1)
SERIAL_DMA_VECTOR(2)
SERIAL_DMA_VECTOR(3)
SERIAL_DMA_VECTOR(4)
SERIAL_DMA_VECTOR(5)
SERIAL_DMA_VECTOR(6)
SERIAL_DMA_VECTOR(7)

static void (*const serial_dma_vectors[SERIAL_DMA_MAX_UARTS])(void) = {
    serial_dma_uart_irq_0, serial_dma_uart_irq_1, serial_dma_uart_irq_2, serial_dma_uart_irq_3,
    serial_dma_uart_irq_4, serial_dma_uart_irq_5, serial_dma_uart_irq_6, serial_dma_uart_irq_7,
};

static void serial_dma_tx_irq(void *context)
{
    serial_dma_t *obj = (serial_dma_t *)context;
    const uint32_t channel = stm_dma_ll_take_events(&obj->tx);
    uint32_t events = 0;
    if (channel & (STM_DMA_LL_TC | STM_DMA_LL_TE)) {
        events |= SERIAL_DMA_TX_CONSUMED; // after an error the piece is lost: the next one gets a fresh start
    }
    if (channel & STM_DMA_LL_TE) {
        events |= SERIAL_DMA_ERROR;
    }
    if (events != 0) {
        obj->handler(obj->context, events);
    }
}

static void serial_dma_rx_restart(serial_dma_t *obj)
{
    obj->rx_laps = 0;
    stm_dma_ll_start_p2m_circular(&obj->rx, LL_USART_DMA_GetRegAddr(obj->uart, LL_USART_DMA_REG_DATA_RECEIVE), obj->rx_ring,
                                  obj->rx_size, &obj->rx_node);
}

static void serial_dma_rx_irq(void *context)
{
    serial_dma_t *obj = (serial_dma_t *)context;
    const uint32_t channel = stm_dma_ll_take_events(&obj->rx);
    uint32_t events = 0;
    if (channel & STM_DMA_LL_TE) {
        // The channel has stopped: receive on from the ring's start
        if (obj->rx_running) {
            serial_dma_rx_restart(obj);
        }
        events |= SERIAL_DMA_ERROR | SERIAL_DMA_RX_RESTART;
    } else {
        if (channel & STM_DMA_LL_TC) {
            obj->rx_laps = obj->rx_laps + 1;
        }
        if (channel & (STM_DMA_LL_TC | STM_DMA_LL_HT)) {
            events |= SERIAL_DMA_RX; // a long burst without a gap: read before the ring wraps
        }
    }
    if (events != 0) {
        obj->handler(obj->context, events);
    }
}

// ---- Set-up --------------------------------------------------------------------------------------------------------

static IRQn_Type serial_dma_irqn(UARTName uart)
{
    switch (uart) {
#if defined(USART1_BASE)
        case UART_1:
            return USART1_IRQn;
#endif
#if defined(USART2_BASE)
        case UART_2:
            return USART2_IRQn;
#endif
#if defined(USART3_BASE)
        case UART_3:
            return USART3_IRQn;
#endif
#if defined(UART4_BASE)
        case UART_4:
            return UART4_IRQn;
#endif
#if defined(UART5_BASE)
        case UART_5:
            return UART5_IRQn;
#endif
#if defined(USART6_BASE)
        case UART_6:
            return USART6_IRQn;
#endif
#if defined(LPUART1_BASE)
        case LPUART_1:
            return LPUART1_IRQn;
#endif
        default:
            return (IRQn_Type)0;
    }
}

static bool serial_dma_link_valid(const DMALinkInfo *link)
{
    return link->dmaIdx != 0;
}

static void serial_dma_release(serial_dma_t *obj)
{
    if (obj->tx_handle.hdma != NULL) {
        stm_dma_ll_stop(&obj->tx);
        stm_free_dma_link(&obj->tx_handle); // the hook goes with it
    }
    if (obj->rx_handle.hdma != NULL) {
        stm_dma_ll_stop(&obj->rx);
        stm_free_dma_link(&obj->rx_handle);
    }
}

int serial_dma_attach(serial_dma_t *obj, serial_t *serial, bool transmit, uint8_t *rx_ring, uint32_t rx_size,
                      serial_dma_handler_t handler, void *context)
{
    struct serial_s *obj_s = SERIAL_S(serial);
    memset(obj, 0, sizeof(*obj));
    const int8_t index = get_uart_index(obj_s->uart);
    if (index < 0 || (size_t)index >= SERIAL_DMA_LINKS || handler == NULL || rx_size > 0xFFFFu) {
        return -1;
    }
    const bool receive = rx_ring != NULL && rx_size > 0;
    if ((transmit && !serial_dma_link_valid(&UARTTxDMALinks[index])) ||
            (receive && !serial_dma_link_valid(&UARTRxDMALinks[index]))) {
        return -1; // no link known for this UART
    }
    core_util_critical_section_enter();
    const bool taken = serial_dma_attached[index] != NULL;
    if (!taken) {
        serial_dma_attached[index] = obj; // reserved; its vector is not set yet
    }
    core_util_critical_section_exit();
    if (taken) {
        return -3;
    }

    obj->uart = (USART_TypeDef *)obj_s->uart;
    obj->irq = serial_dma_irqn(obj_s->uart);
    obj->index = index;
    obj->transmit = transmit;
    obj->rx_ring = receive ? rx_ring : NULL;
    obj->rx_size = receive ? rx_size : 0;
    obj->handler = handler;
    obj->context = context;

    // Byte wide both sides, memory incrementing, the data register not; DMA_NORMAL - stm_dma_ll makes it circular
    // Each channel's LL view as soon as it is claimed: serial_dma_release() stops a claimed channel through it, also
    // when the other claim fails
    if (transmit) {
        obj->tx_handle = stm_claim_dma_link(&UARTTxDMALinks[index], DMA_MEMORY_TO_PERIPH, false, true, 1, 1, DMA_NORMAL);
        if (obj->tx_handle.hdma != NULL) {
            obj->tx = stm_dma_ll_from_handle(&obj->tx_handle);
        }
    }
    if (receive) {
        obj->rx_handle = stm_claim_dma_link(&UARTRxDMALinks[index], DMA_PERIPH_TO_MEMORY, false, true, 1, 1, DMA_NORMAL);
        if (obj->rx_handle.hdma != NULL) {
            obj->rx = stm_dma_ll_from_handle(&obj->rx_handle);
        }
    }
    if ((transmit && obj->tx_handle.hdma == NULL) || (receive && obj->rx_handle.hdma == NULL)) {
        serial_dma_release(obj);
        serial_dma_attached[index] = NULL;
        return -2; // another user's, or none free
    }
    if (transmit) {
        stm_set_dma_irq_hook(&obj->tx_handle, serial_dma_tx_irq, obj);
    }
    if (receive) {
        stm_set_dma_irq_hook(&obj->rx_handle, serial_dma_rx_irq, obj);
    }
    serial_dma_reapply(obj);
    return 0;
}

void serial_dma_detach(serial_dma_t *obj)
{
    if (obj->uart == NULL) {
        return;
    }
    serial_dma_suspend(obj);
    USART_TypeDef *uart = obj->uart;
    LL_USART_DisableIT_IDLE(uart);
    LL_USART_DisableIT_TC(uart);
    LL_USART_DisableDMAReq_TX(uart);
    LL_USART_DisableDMAReq_RX(uart);
    serial_dma_release(obj);
    serial_dma_attached[obj->index] = NULL;
    obj->uart = NULL;
}

void serial_dma_reapply(serial_dma_t *obj)
{
    USART_TypeDef *uart = obj->uart;
    const bool receive = obj->rx_ring != NULL;
    // OVRDIS only with the UART off. The DMA reads every byte; an overrun would stop reception for nothing.
    LL_USART_Disable(uart);
    if (obj->transmit) {
        LL_USART_EnableDMAReq_TX(uart);
    }
    if (receive) {
        LL_USART_EnableDMAReq_RX(uart);
        LL_USART_DisableOverrunDetect(uart);
    }
    LL_USART_Enable(uart);
    if (receive) {
        LL_USART_ClearFlag_IDLE(uart);
        LL_USART_EnableIT_IDLE(uart);
    }
    NVIC_SetVector(obj->irq, (uint32_t)serial_dma_vectors[obj->index]);
    NVIC_EnableIRQ(obj->irq);
}

void serial_dma_suspend(serial_dma_t *obj)
{
    NVIC_DisableIRQ(obj->irq);
    if (obj->tx_handle.hdma != NULL) {
        stm_dma_ll_stop(&obj->tx);
    }
    serial_dma_rx_stop(obj);
    NVIC_ClearPendingIRQ(obj->irq);
}

// ---- Transmit ------------------------------------------------------------------------------------------------------

int serial_dma_tx_start(serial_dma_t *obj, const uint8_t *data, uint32_t length)
{
    if (obj->tx_handle.hdma == NULL || length == 0 || length > 0xFFFFu) {
        return -1;
    }
    stm_dma_ll_start_m2p(&obj->tx, LL_USART_DMA_GetRegAddr(obj->uart, LL_USART_DMA_REG_DATA_TRANSMIT), data, length);
    return 0;
}

void serial_dma_tx_notify_idle(serial_dma_t *obj)
{
    LL_USART_EnableIT_TC(obj->uart);
}

void serial_dma_tx_stop(serial_dma_t *obj)
{
    if (obj->tx_handle.hdma != NULL) {
        stm_dma_ll_stop(&obj->tx);
    }
    LL_USART_DisableIT_TC(obj->uart);
}

void serial_dma_tx_finish(serial_dma_t *obj)
{
    if (obj->tx_handle.hdma == NULL) {
        return;
    }
    for (uint32_t n = 0; n < SERIAL_DMA_POLL_LIMIT && stm_dma_ll_busy(&obj->tx); n++) {
    }
    serial_dma_tx_stop(obj); // its interrupt cleared: the caller goes on by hand
}

void serial_dma_tx_polled(serial_dma_t *obj, const uint8_t *data, uint32_t length)
{
    USART_TypeDef *uart = obj->uart;
    for (uint32_t ix = 0; ix < length; ix++) {
        for (uint32_t n = 0; n < SERIAL_DMA_POLL_LIMIT && !SERIAL_DMA_TX_EMPTY(uart); n++) {
        }
        LL_USART_TransmitData8(uart, data[ix]);
    }
    for (uint32_t n = 0; n < SERIAL_DMA_POLL_LIMIT && !LL_USART_IsActiveFlag_TC(uart); n++) {
    }
}

// ---- Receive -------------------------------------------------------------------------------------------------------

void serial_dma_rx_start(serial_dma_t *obj)
{
    if (obj->rx_handle.hdma == NULL) {
        return;
    }
    core_util_critical_section_enter();
    serial_dma_rx_restart(obj);
    obj->rx_running = true;
    core_util_critical_section_exit();
}

void serial_dma_rx_stop(serial_dma_t *obj)
{
    if (obj->rx_handle.hdma == NULL) {
        return;
    }
    core_util_critical_section_enter();
    obj->rx_running = false;
    stm_dma_ll_stop(&obj->rx);
    core_util_critical_section_exit();
}

uint32_t serial_dma_rx_position(serial_dma_t *obj, uint32_t *laps)
{
    if (!obj->rx_running) {
        *laps = obj->rx_laps;
        return 0;
    }
    uint32_t remaining = stm_dma_ll_remaining(&obj->rx);
    if (remaining == 0 || remaining > obj->rx_size) {
        remaining = obj->rx_size; // reloaded this instant
    }
    *laps = obj->rx_laps + ((stm_dma_ll_events(&obj->rx) & STM_DMA_LL_TC) ? 1u : 0u);
    return obj->rx_size - remaining;
}

#endif // DEVICE_SERIAL_DMA
