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

#ifndef MBED_OS_STM_DMA_LL_H
#define MBED_OS_STM_DMA_LL_H

/* A claimed DMA channel driven by its registers, through ST's LL - for drivers that start a transfer per write or
 * per interrupt and want none of the HAL's bookkeeping there (locks, states, callbacks through a handle).
 *
 * The channel is claimed and configured once with stm_claim_dma_link() (HAL_DMA_Init: request, direction, widths,
 * increments - DMA_NORMAL), its interrupt taken over with stm_set_dma_irq_hook(); from then on only these
 * functions touch it. One implementation per DMA IP:
 *
 *  - v2 (STM32L4 ...): channels with a fixed request, CNDTR/CMAR/CPAR, flags in the controller's ISR/IFCR;
 *    circular by CCR.CIRC.
 *  - v3 (STM32U5 GPDMA): CBR1/CSAR/CDAR, flags in the channel's CSR/CFCR; no circular mode - a circular transfer
 *    is a linked list of one item that reloads the block length and the destination and links back to itself.
 *
 * Every function is inline; the caller keeps them in its interrupt context or a critical section as needed.
 */

#include <stdbool.h>
#include <stdint.h>

#include "stm_dma_utils.h"

#if defined(TARGET_STM32L4)
#include "stm32l4xx_ll_dma.h"
#define STM_DMA_LL_SUPPORTED 1
#elif defined(TARGET_STM32U5)
#include "stm32u5xx_ll_dma.h"
#define STM_DMA_LL_SUPPORTED 1
#else
#define STM_DMA_LL_SUPPORTED 0
#endif

#if STM_DMA_LL_SUPPORTED

#if defined(__cplusplus)
extern "C" {
#endif

/// Channel events, as stm_dma_ll_take_events() returns them.
#define STM_DMA_LL_TC 1u ///< transfer (v3: block) complete
#define STM_DMA_LL_HT 2u ///< half of it done
#define STM_DMA_LL_TE 4u ///< transfer error (v3: also a link or setting error); the channel has stopped

// stm_dma_ll_t and stm_dma_ll_node_t are in stm_dma_utils.h, so objects.h can hold them without the LL header.

/// Bounds a polled wait: a channel that stopped must not hang a caller with interrupts off for ever.
#define STM_DMA_LL_POLL_LIMIT 1000000u

static inline stm_dma_ll_t stm_dma_ll_from_handle(DMAHandlePointer const *handle)
{
    stm_dma_ll_t ll;
#if defined(DMA_IP_VERSION_V2)
    ll.dma = (handle->dmaIdx == 2) ? DMA2 : DMA1;
    ll.channel = (uint32_t)handle->channelIdx - 1u; // LL_DMA_CHANNEL_1 is 0
#else
#if defined(GPDMA2)
    ll.dma = (handle->dmaIdx == 2) ? GPDMA2 : GPDMA1;
#else
    ll.dma = GPDMA1;
#endif
    ll.channel = (uint32_t)handle->channelIdx; // LL_DMA_CHANNEL_0 is 0
#endif
    return ll;
}

#if defined(DMA_IP_VERSION_V2)

// The controller's ISR/IFCR hold four bits a channel: GIF, TCIF, HTIF, TEIF.
static inline uint32_t stm_dma_ll_shift(stm_dma_ll_t const *ll)
{
    return 4u * ll->channel;
}

static inline void stm_dma_ll_clear(stm_dma_ll_t const *ll)
{
    WRITE_REG(ll->dma->IFCR, 0xFu << stm_dma_ll_shift(ll));
}

static inline void stm_dma_ll_interrupts(stm_dma_ll_t const *ll, bool half)
{
    LL_DMA_EnableIT_TC(ll->dma, ll->channel);
    LL_DMA_EnableIT_TE(ll->dma, ll->channel);
    if (half) {
        LL_DMA_EnableIT_HT(ll->dma, ll->channel);
    } else {
        LL_DMA_DisableIT_HT(ll->dma, ll->channel);
    }
}

/// The channel off at once (v2 stops on EN = 0), its flags cleared.
static inline void stm_dma_ll_stop(stm_dma_ll_t const *ll)
{
    LL_DMA_DisableChannel(ll->dma, ll->channel);
    stm_dma_ll_clear(ll);
}

/// @p length bytes from @p memory to the peripheral register at @p peripheral, once; TC (and TE) interrupts.
static inline void stm_dma_ll_start_m2p(stm_dma_ll_t const *ll, uint32_t peripheral, const void *memory, uint32_t length)
{
    LL_DMA_DisableChannel(ll->dma, ll->channel);
    LL_DMA_SetMode(ll->dma, ll->channel, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphAddress(ll->dma, ll->channel, peripheral);
    LL_DMA_SetMemoryAddress(ll->dma, ll->channel, (uint32_t)memory);
    LL_DMA_SetDataLength(ll->dma, ll->channel, length);
    stm_dma_ll_clear(ll);
    stm_dma_ll_interrupts(ll, false);
    LL_DMA_EnableChannel(ll->dma, ll->channel);
}

/// From the peripheral register at @p peripheral into @p memory, round and round; HT, TC (and TE) interrupts.
/// @p node is not used on v2.
static inline void stm_dma_ll_start_p2m_circular(stm_dma_ll_t const *ll, uint32_t peripheral, void *memory, uint32_t length,
                                                 stm_dma_ll_node_t *node)
{
    (void)node;
    LL_DMA_DisableChannel(ll->dma, ll->channel);
    LL_DMA_SetMode(ll->dma, ll->channel, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphAddress(ll->dma, ll->channel, peripheral);
    LL_DMA_SetMemoryAddress(ll->dma, ll->channel, (uint32_t)memory);
    LL_DMA_SetDataLength(ll->dma, ll->channel, length);
    stm_dma_ll_clear(ll);
    stm_dma_ll_interrupts(ll, true);
    LL_DMA_EnableChannel(ll->dma, ll->channel);
}

/// Items still to transfer in the current block (a circular one reloads it at 0).
static inline uint32_t stm_dma_ll_remaining(stm_dma_ll_t const *ll)
{
    return LL_DMA_GetDataLength(ll->dma, ll->channel);
}

/// STM_DMA_LL_* set now, not cleared.
static inline uint32_t stm_dma_ll_events(stm_dma_ll_t const *ll)
{
    const uint32_t bits = READ_REG(ll->dma->ISR) >> stm_dma_ll_shift(ll);
    return ((bits & DMA_ISR_TCIF1) ? STM_DMA_LL_TC : 0u) | ((bits & DMA_ISR_HTIF1) ? STM_DMA_LL_HT : 0u) |
           ((bits & DMA_ISR_TEIF1) ? STM_DMA_LL_TE : 0u);
}

/// STM_DMA_LL_* set now, and cleared - the channel's interrupt.
static inline uint32_t stm_dma_ll_take_events(stm_dma_ll_t const *ll)
{
    const uint32_t events = stm_dma_ll_events(ll);
    stm_dma_ll_clear(ll);
    return events;
}

/// A one-time transfer still moving data.
static inline bool stm_dma_ll_busy(stm_dma_ll_t const *ll)
{
    return LL_DMA_IsEnabledChannel(ll->dma, ll->channel) && LL_DMA_GetDataLength(ll->dma, ll->channel) != 0u;
}

#else // DMA_IP_VERSION_V3

#define STM_DMA_LL_CSR_EVENTS (DMA_CSR_TCF | DMA_CSR_HTF | DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_SUSPF | DMA_CSR_TOF)

static inline DMA_Channel_TypeDef *stm_dma_ll_registers(stm_dma_ll_t const *ll)
{
    return (DMA_Channel_TypeDef *)((uint32_t)ll->dma + LL_DMA_CH_OFFSET_TAB[ll->channel]);
}

static inline void stm_dma_ll_clear(stm_dma_ll_t const *ll)
{
    WRITE_REG(stm_dma_ll_registers(ll)->CFCR, STM_DMA_LL_CSR_EVENTS);
}

static inline void stm_dma_ll_interrupts(stm_dma_ll_t const *ll, bool half)
{
    LL_DMA_EnableIT_TC(ll->dma, ll->channel);
    LL_DMA_EnableIT_DTE(ll->dma, ll->channel);
    LL_DMA_EnableIT_ULE(ll->dma, ll->channel);
    LL_DMA_EnableIT_USE(ll->dma, ll->channel);
    if (half) {
        LL_DMA_EnableIT_HT(ll->dma, ll->channel);
    } else {
        LL_DMA_DisableIT_HT(ll->dma, ll->channel);
    }
}

/// The channel stopped: GPDMA ignores EN = 0 - it is suspended, then reset. Its flags cleared.
static inline void stm_dma_ll_stop(stm_dma_ll_t const *ll)
{
    if (LL_DMA_IsEnabledChannel(ll->dma, ll->channel)) {
        LL_DMA_SuspendChannel(ll->dma, ll->channel);
        for (uint32_t n = 0; n < STM_DMA_LL_POLL_LIMIT && !LL_DMA_IsActiveFlag_SUSP(ll->dma, ll->channel); n++) {
        }
    }
    LL_DMA_ResetChannel(ll->dma, ll->channel); // EN and SUSP cleared, the configuration kept
    stm_dma_ll_clear(ll);
}

static inline void stm_dma_ll_start_m2p(stm_dma_ll_t const *ll, uint32_t peripheral, const void *memory, uint32_t length)
{
    if (LL_DMA_IsEnabledChannel(ll->dma, ll->channel)) {
        stm_dma_ll_stop(ll);
    }
    LL_DMA_ConfigLinkUpdate(ll->dma, ll->channel, 0u, 0u); // one block, then idle
    LL_DMA_SetBlkDataLength(ll->dma, ll->channel, length);
    LL_DMA_SetSrcAddress(ll->dma, ll->channel, (uint32_t)memory);
    LL_DMA_SetDestAddress(ll->dma, ll->channel, peripheral);
    stm_dma_ll_clear(ll);
    stm_dma_ll_interrupts(ll, false);
    LL_DMA_EnableChannel(ll->dma, ll->channel);
}

static inline void stm_dma_ll_start_p2m_circular(stm_dma_ll_t const *ll, uint32_t peripheral, void *memory, uint32_t length,
                                                 stm_dma_ll_node_t *node)
{
    if (LL_DMA_IsEnabledChannel(ll->dma, ll->channel)) {
        stm_dma_ll_stop(ll);
    }
    // At the block's end the channel loads CBR1 (the length again), CDAR (the ring's start) and CLLR (this item)
    const uint32_t update = DMA_CLLR_UB1 | DMA_CLLR_UDA | DMA_CLLR_ULL;
    node->cbr1 = length & DMA_CBR1_BNDT;
    node->cdar = (uint32_t)memory;
    node->cllr = update | ((uint32_t)node & DMA_CLLR_LA);
    LL_DMA_SetLinkAllocatedPort(ll->dma, ll->channel, LL_DMA_LINK_ALLOCATED_PORT0); // port 0 sees all of SRAM
    LL_DMA_SetLinkedListBaseAddr(ll->dma, ll->channel, (uint32_t)node);
    LL_DMA_ConfigLinkUpdate(ll->dma, ll->channel, update, (uint32_t)node);
    LL_DMA_SetBlkDataLength(ll->dma, ll->channel, length);
    LL_DMA_SetSrcAddress(ll->dma, ll->channel, peripheral);
    LL_DMA_SetDestAddress(ll->dma, ll->channel, (uint32_t)memory);
    stm_dma_ll_clear(ll);
    stm_dma_ll_interrupts(ll, true);
    LL_DMA_EnableChannel(ll->dma, ll->channel);
}

static inline uint32_t stm_dma_ll_remaining(stm_dma_ll_t const *ll)
{
    return LL_DMA_GetBlkDataLength(ll->dma, ll->channel);
}

static inline uint32_t stm_dma_ll_events(stm_dma_ll_t const *ll)
{
    const uint32_t bits = READ_REG(stm_dma_ll_registers(ll)->CSR);
    return ((bits & DMA_CSR_TCF) ? STM_DMA_LL_TC : 0u) | ((bits & DMA_CSR_HTF) ? STM_DMA_LL_HT : 0u) |
           ((bits & (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF)) ? STM_DMA_LL_TE : 0u);
}

static inline uint32_t stm_dma_ll_take_events(stm_dma_ll_t const *ll)
{
    const uint32_t events = stm_dma_ll_events(ll);
    stm_dma_ll_clear(ll);
    return events;
}

/// A one-time transfer still moving data: GPDMA disables the channel itself at the end of the last block.
static inline bool stm_dma_ll_busy(stm_dma_ll_t const *ll)
{
    return LL_DMA_IsEnabledChannel(ll->dma, ll->channel) != 0u;
}

#endif // DMA_IP_VERSION_V3

#if defined(__cplusplus)
}
#endif

#endif // STM_DMA_LL_SUPPORTED

#endif // MBED_OS_STM_DMA_LL_H
