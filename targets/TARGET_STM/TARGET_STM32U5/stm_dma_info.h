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

// STM32U5+ devices.
// On this device, the DMA channels may be chosen arbitrarily.

/// Mapping from SPI index to DMA link info for Tx
static const DMALinkInfo SPITxDMALinks[] = {
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_SPI1_TX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_SPI2_TX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_SPI3_TX}
};

/// Mapping from SPI index to DMA link info for Rx
static const DMALinkInfo SPIRxDMALinks[] = {
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_SPI1_RX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_SPI2_RX},
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_SPI3_RX}
};

/// Mapping from OSPI index to DMA link info
static const DMALinkInfo OSPIDMALinks[] = {
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_OCTOSPI1},
#if defined(OCTOSPI2)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_OCTOSPI2}
#endif
};

/// Mapping from UART index (get_uart_index()) to DMA link info for Tx (serial_dma_api.h)
static const DMALinkInfo UARTTxDMALinks[] = {
#if defined(USART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART1_TX},
#endif
#if defined(USART2_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART2_TX},
#endif
#if defined(USART3_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART3_TX},
#endif
#if defined(UART4_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_UART4_TX},
#endif
#if defined(UART5_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_UART5_TX},
#endif
#if defined(USART6_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART6_TX},
#endif
#if defined(LPUART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_LPUART1_TX},
#endif
};

/// Mapping from UART index (get_uart_index()) to DMA link info for Rx (serial_dma_api.h)
static const DMALinkInfo UARTRxDMALinks[] = {
#if defined(USART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART1_RX},
#endif
#if defined(USART2_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART2_RX},
#endif
#if defined(USART3_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART3_RX},
#endif
#if defined(UART4_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_UART4_RX},
#endif
#if defined(UART5_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_UART5_RX},
#endif
#if defined(USART6_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_USART6_RX},
#endif
#if defined(LPUART1_BASE)
        {MBED_ANY_GPDMA_MODULE, MBED_ANY_DMA_CHANNEL, GPDMA1_REQUEST_LPUART1_RX},
#endif
};

#endif //MBED_OS_STM_DMA_INFO_H
