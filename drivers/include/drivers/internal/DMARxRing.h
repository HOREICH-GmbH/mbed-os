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
#ifndef MBED_DMA_RX_RING_H
#define MBED_DMA_RX_RING_H

#include <stddef.h>
#include <stdint.h>

namespace mbed {
namespace internal {

/** DMASerial's receive side without the hardware: a ring a circular DMA writes round and round, and a reader behind
 *  it. Where the DMA is, is two numbers - its laps (the ring's end passed) and its position in the current lap - so
 *  what was written is laps x size + position, and what is to be read the difference to what was read. A writer more
 *  than a ring ahead has overwritten bytes not yet read: the reader skips to the oldest still there and the loss is
 *  counted. Free of mbed and the MCU (host tests); called with interrupts off.
 */
class DMARxRing {
public:
    DMARxRing(uint8_t *buffer, size_t size) :
        _buffer(buffer),
        _size(buffer != nullptr ? size : 0)
    {
    }

    /** The DMA starts again at the ring's start, nothing to read. */
    void reset();

    /** Where the DMA is: @p laps rings completed, @p position bytes into the current one. */
    void update(uint32_t laps, size_t position);

    size_t available() const
    {
        return _written - _read;
    }

    /** Up to @p size bytes into @p out, the oldest first; the count. */
    size_t read(uint8_t *out, size_t size);

    /** Bytes overwritten before they were read, since the start. */
    uint32_t overruns() const
    {
        return _overruns;
    }

    size_t size() const
    {
        return _size;
    }

private:
    uint8_t *_buffer;
    size_t _size;
    uint32_t _written = 0; ///< bytes the DMA has written, wrapping at 2^32 like _read
    uint32_t _read = 0;
    uint32_t _overruns = 0;
};

} // namespace internal
} // namespace mbed

#endif // MBED_DMA_RX_RING_H
