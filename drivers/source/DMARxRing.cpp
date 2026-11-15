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
    _read_at = 0;
    _ends_first = 0;
    _ends_count = 0;
    _last_end = 0;
    _scanned = 0;
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
        advance_read(lost);
    }
}

void DMARxRing::advance_read(uint32_t count)
{
    _read += count;
    _read_at = (_read_at + (size_t)(count % (uint32_t)_size)) % _size;
}

size_t DMARxRing::read(uint8_t *out, size_t size)
{
    const size_t wanted = size < available() ? size : available();
    if (wanted == 0) {
        return 0;
    }
    const size_t at = _read_at;
    const size_t first = wanted < _size - at ? wanted : _size - at;
    memcpy(out, _buffer + at, first);
    memcpy(out + first, _buffer, wanted - first);
    advance_read((uint32_t)wanted);
    return wanted;
}

void DMARxRing::frame_end(uint32_t laps, size_t position)
{
    if (_size == 0) {
        return;
    }
    end_at(laps * (uint32_t)_size + (uint32_t)position);
}

void DMARxRing::frame_end_at_match(uint32_t laps, size_t position, uint8_t match)
{
    if (_size == 0) {
        return;
    }
    const uint32_t end = laps * (uint32_t)_size + (uint32_t)position;
    // From the first byte not looked at - not before the last end, the reader, or the oldest byte still in the ring
    uint32_t from = _scanned;
    if ((int32_t)(_last_end - from) > 0) {
        from = _last_end;
    }
    if ((int32_t)(_read - from) > 0) {
        from = _read;
    }
    if ((int32_t)((end - (uint32_t)_size) - from) > 0) {
        from = end - (uint32_t)_size;
    }
    for (uint32_t at = from; (int32_t)(end - at) > 0; at++) {
        if (_buffer[index_of(at)] == match) {
            end_at(at + 1);
        }
    }
    if ((int32_t)(end - _scanned) > 0) {
        _scanned = end;
    }
}

void DMARxRing::end_at(uint32_t end)
{
    if ((int32_t)(end - _last_end) <= 0 || (int32_t)(end - _read) <= 0) {
        return; // nothing since the last end, or nothing the reader has not passed: no frame
    }
    _last_end = end;
    if (_ends_count == max_frames) {
        // Full: the oldest end goes, its frame and the next one read as one
        _ends_first = (_ends_first + 1) % max_frames;
        _ends_count--;
        _frames_merged++;
    }
    _ends[(_ends_first + _ends_count) % max_frames] = end;
    _ends_count++;
}

void DMARxRing::drop_read_frames()
{
    while (_ends_count > 0 && (int32_t)(_ends[_ends_first] - _read) <= 0) {
        _ends_first = (_ends_first + 1) % max_frames;
        _ends_count--;
    }
}

size_t DMARxRing::frames() const
{
    size_t whole = 0;
    for (size_t i = 0; i < _ends_count; i++) {
        const uint32_t end = _ends[(_ends_first + i) % max_frames];
        if ((int32_t)(end - _read) > 0 && (int32_t)(end - _written) <= 0) {
            whole++;
        }
    }
    return whole;
}

size_t DMARxRing::read_frame(uint8_t *out, size_t size, bool *truncated)
{
    drop_read_frames();
    if (truncated != nullptr) {
        *truncated = false;
    }
    if (_ends_count == 0 || (int32_t)(_ends[_ends_first] - _written) > 0) {
        return 0; // none, or its end not written yet (update() not called since)
    }
    const uint32_t end = _ends[_ends_first];
    _ends_first = (_ends_first + 1) % max_frames;
    _ends_count--;
    const size_t length = end - _read;
    const size_t copied = read(out, length < size ? length : size);
    if (copied < length) {
        advance_read(end - _read); // the rest of a frame longer than the caller's buffer
        if (truncated != nullptr) {
            *truncated = true;
        }
    }
    return copied;
}

} // namespace internal
} // namespace mbed
