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

#ifndef MBED_SERIAL_DMA_OBJECT_H
#define MBED_SERIAL_DMA_OBJECT_H

#if DEVICE_SERIAL_DMA

#include <stdbool.h>
#include <stdint.h>

#include "stm_dma_utils.h"

/// One UART by DMA (hal/serial_dma_api.h, stm_serial_dma.c). Included by the families' objects.h.
struct serial_dma_s {
    USART_TypeDef *uart;
    IRQn_Type irq;
    int8_t index;                    ///< get_uart_index(): its links, its vector
    bool transmit;
    DMAHandlePointer tx_handle;      ///< claimed, hdma NULL when not
    DMAHandlePointer rx_handle;
    stm_dma_ll_t tx;
    stm_dma_ll_t rx;
    stm_dma_ll_node_t rx_node;       ///< GPDMA's circular item
    uint8_t *rx_ring;
    uint32_t rx_size;
    volatile uint32_t rx_laps;       ///< rings completed since serial_dma_rx_start()
    bool rx_running;
    uint8_t frame_mode;              ///< serial_dma_frame_t
    uint32_t frame_value;            ///< its bit times or character
    uint32_t frame_position;         ///< where the receive DMA was when the last frame ended
    uint32_t frame_laps;
    void (*handler)(void *context, uint32_t events);
    void *context;
};

#endif // DEVICE_SERIAL_DMA

#endif // MBED_SERIAL_DMA_OBJECT_H
