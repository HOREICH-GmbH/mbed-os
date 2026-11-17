/* mbed Microcontroller Library
 * Copyright (c) 2016-2023 STMicroelectronics
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

#ifndef MBED_OS_STM_DMA_INFO_H
#define MBED_OS_STM_DMA_INFO_H

#include "cmsis.h"
#include "stm_dma_utils.h"

#ifdef DMAMUX1

// STM32L4+ devices, with DMAMUX feature.
// On this device, the DMA channels may be chosen arbitrarily.

/// Mapping from SPI index to DMA link info for Tx
static const DMALinkInfo SPITxDMALinks[] = {
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_SPI1_TX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_SPI2_TX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_SPI3_TX}
};

/// Mapping from SPI index to DMA link info for Rx
static const DMALinkInfo SPIRxDMALinks[] = {
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_SPI1_RX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_SPI2_RX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_SPI3_RX}
};

/// Mapping from OSPI index to DMA link info
static const DMALinkInfo OSPIDMALinks[] = {
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_OCTOSPI1},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_OCTOSPI2},
};

/// Mapping from UART index (get_uart_index()) to DMA link info for Tx (serial_dma_api.h)
static const DMALinkInfo UARTTxDMALinks[] = {
#if defined(USART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_USART1_TX},
#endif
#if defined(USART2_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_USART2_TX},
#endif
#if defined(USART3_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_USART3_TX},
#endif
#if defined(UART4_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_UART4_TX},
#endif
#if defined(UART5_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_UART5_TX},
#endif
#if defined(LPUART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_LPUART1_TX},
#endif
};

/// Mapping from UART index (get_uart_index()) to DMA link info for Rx (serial_dma_api.h)
static const DMALinkInfo UARTRxDMALinks[] = {
#if defined(USART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_USART1_RX},
#endif
#if defined(USART2_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_USART2_RX},
#endif
#if defined(USART3_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_USART3_RX},
#endif
#if defined(UART4_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_UART4_RX},
#endif
#if defined(UART5_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_UART5_RX},
#endif
#if defined(LPUART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, DMA_REQUEST_LPUART1_RX},
#endif
};

#else


// Base model STM32L4 devices, with fixed DMA line mapping
// See STM32L4 reference manual Tables 41 and 42.

/// Mapping from SPI index to DMA link info for Tx
static const DMALinkInfo SPITxDMALinks[] = {
        {1, 3, 1}, // SPI1 Tx is DMA1 Ch3 Request 1
        {1, 5, 1}, // SPI2 Tx is DMA1 Ch5 Request 1
        {2, 2, 3}  // SPI3 Tx is DMA2 Ch2 Request 3
};

/// Mapping from SPI index to DMA link info for Rx
static const DMALinkInfo SPIRxDMALinks[] = {
        {1, 2, 1}, // SPI1 Rx is DMA1 Ch2 Request 1
        {1, 4, 1}, // SPI2 Rx is DMA1 Ch4 Request 1
        {2, 1, 3}  // SPI3 Rx is DMA2 Ch1 Request 3
};

/// Mapping from QSPI index to DMA link info
static const DMALinkInfo QSPIDMALinks[] = {
        {2, 7, 3}, // QUADSPI is DMA2 Ch7 Request 3
};

// The UARTs' links: RM0351 Rev 11 Tables 46 and 47 - the L47x/L48x/L49x/L4Ax. Other parts (RM0394, RM0392)
// get {0, 0, 0}, no DMA: serial_dma_attach() refuses and the user falls back, until their manual is checked.
#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx) || \
    defined(STM32L496xx) || defined(STM32L4A6xx)
#define STM_UART_DMA_LINK(dma, channel, request) {dma, channel, request}
#else
#define STM_UART_DMA_LINK(dma, channel, request) {0, 0, 0}
#endif

/// Mapping from UART index (get_uart_index()) to DMA link info for Tx (serial_dma_api.h). Where a request has two
/// channels, the one that does not collide with the links above (or the one least likely used) is taken.
static const DMALinkInfo UARTTxDMALinks[] = {
#if defined(USART1_BASE)
        STM_UART_DMA_LINK(2, 6, 2), // USART1 Tx is DMA2 Ch6 Request 2 (DMA1 Ch4 is SPI2 Rx's)
#endif
#if defined(USART2_BASE)
        STM_UART_DMA_LINK(1, 7, 2), // USART2 Tx is DMA1 Ch7 Request 2
#endif
#if defined(USART3_BASE)
        STM_UART_DMA_LINK(1, 2, 2), // USART3 Tx is DMA1 Ch2 Request 2 (shared with SPI1 Rx)
#endif
#if defined(UART4_BASE)
        STM_UART_DMA_LINK(2, 3, 2), // UART4 Tx is DMA2 Ch3 Request 2
#endif
#if defined(UART5_BASE)
        STM_UART_DMA_LINK(2, 1, 2), // UART5 Tx is DMA2 Ch1 Request 2 (shared with SPI3 Rx)
#endif
#if defined(LPUART1_BASE)
        STM_UART_DMA_LINK(2, 6, 4), // LPUART1 Tx is DMA2 Ch6 Request 4 (shared with USART1 Tx)
#endif
};

/// Mapping from UART index (get_uart_index()) to DMA link info for Rx (serial_dma_api.h)
static const DMALinkInfo UARTRxDMALinks[] = {
#if defined(USART1_BASE)
        STM_UART_DMA_LINK(1, 5, 2), // USART1 Rx is DMA1 Ch5 Request 2 (DMA2 Ch7 is QUADSPI's; Ch5 shared with SPI2 Tx)
#endif
#if defined(USART2_BASE)
        STM_UART_DMA_LINK(1, 6, 2), // USART2 Rx is DMA1 Ch6 Request 2
#endif
#if defined(USART3_BASE)
        STM_UART_DMA_LINK(1, 3, 2), // USART3 Rx is DMA1 Ch3 Request 2 (shared with SPI1 Tx)
#endif
#if defined(UART4_BASE)
        STM_UART_DMA_LINK(2, 5, 2), // UART4 Rx is DMA2 Ch5 Request 2
#endif
#if defined(UART5_BASE)
        STM_UART_DMA_LINK(2, 2, 2), // UART5 Rx is DMA2 Ch2 Request 2 (shared with SPI3 Tx)
#endif
#if defined(LPUART1_BASE)
        STM_UART_DMA_LINK(2, 7, 4), // LPUART1 Rx is DMA2 Ch7 Request 4 (shared with QUADSPI)
#endif
};

#undef STM_UART_DMA_LINK

#endif

#endif //MBED_OS_STM_DMA_INFO_H
