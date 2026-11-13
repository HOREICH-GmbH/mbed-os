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
#ifndef MBED_DMA_TX_RING_H
#define MBED_DMA_TX_RING_H

#include <stddef.h>
#include <stdint.h>

namespace mbed {
namespace internal {

/** DMASerial's transmit side without the hardware: a ring the writers copy into and the DMA sends from, one
 *  contiguous piece at a time. Free of mbed and of the MCU, so it runs on the PC (host tests); DMASerial is the Port.
 *
 *  Every call is made with interrupts off - the writer's critical section, or the DMA's and the UART's own
 *  interrupts - so the state needs no lock.
 *
 *  A write that does not fit is either dropped whole and counted - "[n bytes dropped]" goes out where the bytes were
 *  lost, once there is room (write(), a console) - or taken as far as it fits (write_some(), a modem: nothing may go
 *  missing; the caller waits for room). Nothing here waits, except write_polled(), for when nothing else can run.
 */
class DMATxRing {
public:
    /** The hardware. Called with interrupts off. */
    class Port {
    public:
        virtual ~Port() = default;

        /** Hands @p length bytes at @p data to the DMA. completed() follows, from its interrupt, once they are all
         *  with the UART. */
        virtual void start(const uint8_t *data, size_t length) = 0;

        /** Interrupts off for good: waits until what start() handed out has gone to the UART, and keeps its
         *  interrupt from firing later. */
        virtual void finish() = 0;

        /** Interrupts off for good: @p data out byte by byte, waiting for each, and until the last has left the line. */
        virtual void send_polled(const uint8_t *data, size_t length) = 0;

        /** Everything is with the UART: call idle() once its last byte has left the line. Until then the clock must
         *  stay on. */
        virtual void notify_idle() = 0;

        /** False while bytes are on their way (deep sleep would stop the UART mid-byte), true once the line is quiet. */
        virtual void allow_deep_sleep(bool allow) = 0;
    };

    DMATxRing(Port &port, uint8_t *buffer, size_t size);

    /** Into the ring and on its way; dropped whole (and counted) when it does not fit. Never waits. */
    void write(const uint8_t *data, size_t length);

    /** As much as fits, nothing dropped: the count taken, 0 when the ring is full. */
    size_t write_some(const uint8_t *data, size_t length);

    /** Room in the ring now. */
    size_t free() const
    {
        return _size - _count;
    }

    /** Interrupts are off for good (mbed writes a fatal error that way): what waits in the ring, then @p data, out
     *  now, polled. */
    void write_polled(const uint8_t *data, size_t length);

    /** Everything waiting dropped, the line taken as quiet: the port is being switched off (its DMA already
     *  stopped). Counts stay. */
    void reset();

    /** The DMA's interrupt: the piece handed to it is with the UART. */
    void completed();

    /** The UART's interrupt: its last byte has left the line. */
    void idle();

    /** Bytes waiting, on their way, or still leaving the line. */
    bool busy() const
    {
        return _state != State::Idle;
    }

    /** Bytes waiting in the ring, the piece on its way included. */
    size_t pending() const
    {
        return _count;
    }

    /** Every byte dropped since the start. */
    uint32_t dropped() const
    {
        return _dropped_total;
    }

private:
    enum class State {
        Idle,     ///< nothing to send, the line quiet: deep sleep allowed
        Sending,  ///< a piece with the DMA
        Draining, ///< all with the UART, its last byte leaving the line
    };

    void start_next();
    void append(const uint8_t *data, size_t length);
    bool append_note(); ///< "[n bytes dropped]", if it fits; true when it went in
    void send_polled_ring();

    Port &_port;
    uint8_t *_buffer;
    size_t _size;
    size_t _tail = 0;      ///< the oldest byte not yet with the UART
    size_t _count = 0;     ///< bytes from _tail on, the piece with the DMA included
    size_t _in_flight = 0; ///< the piece with the DMA, from _tail
    State _state = State::Idle;
    uint32_t _dropped_pending = 0; ///< dropped and not yet said
    uint32_t _dropped_total = 0;
};

} // namespace internal
} // namespace mbed

#endif // MBED_DMA_TX_RING_H
