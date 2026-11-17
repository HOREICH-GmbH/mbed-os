/** \addtogroup hal */
/** @{*/
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
#ifndef MBED_SERIAL_DMA_API_H
#define MBED_SERIAL_DMA_API_H

#include "hal/serial_api.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if DEVICE_SERIAL_DMA

/**
 * \defgroup hal_serial_dma Serial by DMA
 *
 * A UART set up by serial_api.h (serial_init(), serial_baud(), serial_set_flow_control(), serial_free()) whose data
 * goes by DMA: a piece at a time out of the caller's memory, and in round and round into the caller's ring. It
 * lives beside the asynchronous serial API - a UART is driven by one or the other, never both.
 *
 * Transmit is two events, so pieces chain without a gap: SERIAL_DMA_TX_CONSUMED when the DMA has handed a piece to
 * the UART (the next may start), SERIAL_DMA_TX_IDLE once its last bit has left the line - asked for with
 * serial_dma_tx_notify_idle(), for the caller that holds deep sleep off while sending.
 * Receive runs until stopped: SERIAL_DMA_RX when the line goes idle after a burst and when the DMA passes the
 * ring's middle and end; where the DMA is, serial_dma_rx_position(). A line error (framing, noise, parity) is
 * cleared and reported, and reception goes on.
 * Frames (optional, serial_dma_rx_frame()): the UART tells where a received frame ends - the line idle for a
 * character, silent for a set number of bit times (receiver timeout: Modbus RTU's 3.5 characters), or a set
 * character received (character match). SERIAL_DMA_RX_FRAME then, with where the DMA was at that moment,
 * serial_dma_rx_frame_end() - captured in the interrupt, so a frame that follows at once is not glued on.
 *
 * Health (serial_dma_check()): the ways reception stops without saying so - the receive channel disabled, the UART's
 * DMA requests or frame interrupts cleared behind the driver's back, its vector taken over - and interrupts that
 * handled nothing, counted and, SERIAL_DMA_STRAY_LIMIT in a row, the UART's interrupt switched off.
 *
 * Every function except attach and detach may be called with interrupts off; the handler is called from interrupt
 * context.
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/** The target's state of one UART by DMA (objects.h) */
typedef struct serial_dma_s serial_dma_t;

#define SERIAL_DMA_TX_CONSUMED (1u << 0) ///< the piece is with the UART: the next one may start
#define SERIAL_DMA_TX_IDLE     (1u << 1) ///< the last bit has left the line (after serial_dma_tx_notify_idle())
#define SERIAL_DMA_RX          (1u << 2) ///< bytes arrived: the line went idle, or the DMA passed the ring's middle or end
#define SERIAL_DMA_RX_RESTART  (1u << 3) ///< the receive DMA failed and started again at the ring's start
#define SERIAL_DMA_ERROR       (1u << 4) ///< a line error or a DMA transfer error, cleared
#define SERIAL_DMA_RX_FRAME    (1u << 5) ///< a frame ended (serial_dma_rx_frame()): where, serial_dma_rx_frame_end()

/** How the end of a received frame is told */
typedef enum {
    SERIAL_DMA_FRAME_NONE = 0, ///< not at all: a stream (the idle line still says SERIAL_DMA_RX)
    SERIAL_DMA_FRAME_IDLE,     ///< the line idle for one character after a byte
    SERIAL_DMA_FRAME_TIMEOUT,  ///< the line silent for a set number of bit times after a byte (receiver timeout)
    SERIAL_DMA_FRAME_MATCH,    ///< a set character received (character match), the frame's last byte
} serial_dma_frame_t;

/** serial_dma_check()'s findings - 0: healthy */
#define SERIAL_DMA_FAULT_RX_STOPPED  (1u << 0) ///< receiving, but the receive DMA channel is disabled
#define SERIAL_DMA_FAULT_DMA_REQUEST (1u << 1) ///< the UART's DMA request bits (CR3 DMAT/DMAR) cleared
#define SERIAL_DMA_FAULT_INTERRUPT   (1u << 2) ///< an interrupt enable (IDLE, line errors, the frame's) or the receiver timeout cleared
#define SERIAL_DMA_FAULT_VECTOR      (1u << 3) ///< the UART's vector not this layer's, or its interrupt off in the NVIC
#define SERIAL_DMA_FAULT_STRAY       (1u << 4) ///< SERIAL_DMA_STRAY_LIMIT interrupts in a row handled nothing: switched off

/** Interrupts in a row that handled nothing before the UART's interrupt is switched off */
#define SERIAL_DMA_STRAY_LIMIT 100u

/** Called from interrupt context with SERIAL_DMA_* events */
typedef void (*serial_dma_handler_t)(void *context, uint32_t events);

/** Take the UART's data over: claim its DMA channels, set the UART up for them, take its interrupt
 *
 * @param obj      the state, the caller's for as long as it is attached
 * @param serial   the UART, set up by serial_init()
 * @param transmit send by DMA
 * @param rx_ring  receive by DMA into this ring, round and round; NULL: no reception
 * @param rx_size  its size in bytes (up to 65535)
 * @param handler  called with the events
 * @param context  passed to @p handler
 * @return 0, or negative: no DMA link for this UART, a channel another user's (or none free), the UART taken
 */
int serial_dma_attach(serial_dma_t *obj, serial_t *serial, bool transmit, uint8_t *rx_ring, uint32_t rx_size,
                      serial_dma_handler_t handler, void *context);

/** Give it back: transfers stopped, channels released, the UART's DMA requests and interrupts off */
void serial_dma_detach(serial_dma_t *obj);

/** The UART's DMA bits and interrupt back on: after every serial_init(), serial_baud(), serial_format() and
 *  serial_set_flow_control() - they set the UART up anew */
void serial_dma_reapply(serial_dma_t *obj);

/** Before serial_free(): both transfers stopped, the UART's interrupt off. serial_dma_reapply() after the next
 *  serial_init() takes it up again */
void serial_dma_suspend(serial_dma_t *obj);

/** Send @p length bytes from @p data (which stay put until SERIAL_DMA_TX_CONSUMED) */
int serial_dma_tx_start(serial_dma_t *obj, const uint8_t *data, uint32_t length);

/** SERIAL_DMA_TX_IDLE once the last bit has left the line */
void serial_dma_tx_notify_idle(serial_dma_t *obj);

/** The transfer under way stopped, its interrupt not to come */
void serial_dma_tx_stop(serial_dma_t *obj);

/** Interrupts off for good: waits (bounded) until the transfer under way is with the UART, then stops it */
void serial_dma_tx_finish(serial_dma_t *obj);

/** Interrupts off for good: @p data out byte by byte, waiting for each, and until the last one has left the line */
void serial_dma_tx_polled(serial_dma_t *obj, const uint8_t *data, uint32_t length);

/** Reception into the ring, from its start */
void serial_dma_rx_start(serial_dma_t *obj);

/** Reception stopped */
void serial_dma_rx_stop(serial_dma_t *obj);

/** Where the receive DMA is, with interrupts off: the position in the ring, and in @p laps the rings completed since
 *  serial_dma_rx_start() - one whose interrupt is still pending included */
uint32_t serial_dma_rx_position(serial_dma_t *obj, uint32_t *laps);

/** How a received frame's end is told: SERIAL_DMA_FRAME_TIMEOUT @p value bit times of silence (1 to 0xFFFFFF),
 *  SERIAL_DMA_FRAME_MATCH @p value the character (0 to 255); ignored for the others. Kept until changed, also across
 *  serial_dma_reapply(). Sets the UART up anew, as serial_dma_reapply() - a byte under way is lost: before traffic.
 *
 * @return 0, or -1: this UART cannot (no receiver timeout on an LPUART), -2: @p value out of range, -3: no reception
 */
int serial_dma_rx_frame(serial_dma_t *obj, serial_dma_frame_t mode, uint32_t value);

/** Where the receive DMA was when the last frame ended, as serial_dma_rx_position() - for SERIAL_DMA_RX_FRAME's
 *  handler, which is called before the next frame can end */
uint32_t serial_dma_rx_frame_end(serial_dma_t *obj, uint32_t *laps);

/** What is wrong with the UART's DMA set-up now: SERIAL_DMA_FAULT_* bits, 0 healthy. Nothing while suspended
 *  (serial_dma_suspend() to the next serial_dma_reapply()) or detached. */
uint32_t serial_dma_check(serial_dma_t *obj);

/** UART interrupts that handled nothing, since attach */
uint32_t serial_dma_stray_interrupts(serial_dma_t *obj);

/** The UART's registers and the receive DMA, one line into @p out (snprintf's return) */
int serial_dma_describe(serial_dma_t *obj, char *out, size_t size);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // DEVICE_SERIAL_DMA

#endif // MBED_SERIAL_DMA_API_H

/** @}*/
