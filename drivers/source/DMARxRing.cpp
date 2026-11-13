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
#include "drivers/internal/DMARxRing.h"

#include <string.h>

namespace mbed {
namespace internal {

void DMARxRing::reset()
{
    _written = 0;
    _read = 0;
}

void DMARxRing::update(uint32_t laps, size_t position)
{
    if (_size == 0) {
        return;
    }
    const uint32_t written = laps * (uint32_t)_size + (uint32_t)position;
    // Never backwards: a position read just after the wrap, its lap not yet counted, is the same lap's start
    if ((int32_t)(written - _written) > 0) {
        _written = written;
    }
    if (_written - _read > _size) {
        // Lapped: what was not read in time is gone - on from the oldest byte still in the ring
        const uint32_t lost = _written - _read - (uint32_t)_size;
        _overruns += lost;
        _read += lost;
    }
}

size_t DMARxRing::read(uint8_t *out, size_t size)
{
    const size_t wanted = size < available() ? size : available();
    if (wanted == 0) {
        return 0;
    }
    const size_t at = _read % _size;
    const size_t first = wanted < _size - at ? wanted : _size - at;
    memcpy(out, _buffer + at, first);
    memcpy(out + first, _buffer, wanted - first);
    _read += (uint32_t)wanted;
    return wanted;
}

} // namespace internal
} // namespace mbed
