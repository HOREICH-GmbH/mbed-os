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
#include "drivers/internal/DMATxRing.h"

#include <string.h>

namespace mbed {
namespace internal {

namespace {

constexpr size_t note_size = 32; // "[4294967295 bytes dropped]\r\n" fits

/** "[n bytes dropped]\r\n" by hand: this runs with interrupts off, where snprintf may not (newlib's formatter takes
 *  locks), and it is no place for a formatter anyway. */
size_t format_note(char (&note)[note_size], uint32_t dropped)
{
    char digits[10];
    size_t count = 0;
    do {
        digits[count++] = (char)('0' + dropped % 10);
        dropped /= 10;
    } while (dropped > 0);
    size_t length = 0;
    note[length++] = '[';
    while (count > 0) {
        note[length++] = digits[--count];
    }
    static constexpr char tail[] = " bytes dropped]\r\n";
    memcpy(note + length, tail, sizeof(tail) - 1);
    return length + sizeof(tail) - 1;
}

size_t min_size(size_t a, size_t b)
{
    return a < b ? a : b;
}

} // namespace

DMATxRing::DMATxRing(Port &port, uint8_t *buffer, size_t size) :
    _port(port),
    _buffer(buffer),
    _size(buffer != nullptr ? size : 0)
{
}

void DMATxRing::append(const uint8_t *data, size_t length)
{
    // The caller made sure it fits; up to the end of the buffer, the rest from its start
    const size_t at = (_tail + _count) % _size;
    const size_t first = min_size(length, _size - at);
    memcpy(_buffer + at, data, first);
    memcpy(_buffer, data + first, length - first);
    _count += length;
}

bool DMATxRing::append_note()
{
    char note[note_size];
    const size_t length = format_note(note, _dropped_pending);
    if (length > _size - _count) {
        return false;
    }
    append(reinterpret_cast<const uint8_t *>(note), length);
    _dropped_pending = 0;
    return true;
}

void DMATxRing::write(const uint8_t *data, size_t length)
{
    if (length == 0 || _size == 0) {
        return;
    }
    // A loss is said where it happened: before the first write that fits again
    if (_dropped_pending > 0) {
        char note[note_size];
        if (format_note(note, _dropped_pending) + length <= _size - _count) {
            append_note();
        }
    }
    if (_dropped_pending > 0 || length > _size - _count) {
        _dropped_pending += (uint32_t)length;
        _dropped_total += (uint32_t)length;
    } else {
        append(data, length);
    }
    start_next();
}

size_t DMATxRing::write_some(const uint8_t *data, size_t length)
{
    const size_t taken = min_size(length, _size - _count);
    if (taken > 0) {
        append(data, taken);
        start_next();
    }
    return taken;
}

void DMATxRing::start_next()
{
    if (_in_flight > 0) {
        return; // completed() goes on
    }
    if (_count == 0 && _dropped_pending > 0) {
        append_note(); // the ring is empty: it fits
    }
    if (_count == 0) {
        if (_state == State::Sending) {
            // All with the UART; the clock stays on until its last byte has left the line
            _state = State::Draining;
            _port.notify_idle();
        }
        return;
    }
    if (_state == State::Idle) {
        _port.allow_deep_sleep(false);
    }
    _state = State::Sending;
    // One contiguous piece: up to the buffer's end, the rest after it
    _in_flight = min_size(_count, _size - _tail);
    _port.start(_buffer + _tail, _in_flight);
}

void DMATxRing::reset()
{
    _tail = 0;
    _count = 0;
    _in_flight = 0;
    _dropped_pending = 0;
    if (_state != State::Idle) {
        _state = State::Idle;
        _port.allow_deep_sleep(true);
    }
}

void DMATxRing::completed()
{
    if (_in_flight == 0) {
        return;
    }
    _tail = (_tail + _in_flight) % _size;
    _count -= _in_flight;
    _in_flight = 0;
    start_next();
}

void DMATxRing::idle()
{
    // Only the end of a drain counts: a transfer started since then ends in a drain of its own
    if (_state == State::Draining && _count == 0) {
        _state = State::Idle;
        _port.allow_deep_sleep(true);
    }
}

void DMATxRing::send_polled_ring()
{
    const size_t first = min_size(_count, _size - _tail);
    _port.send_polled(_buffer + _tail, first);
    _port.send_polled(_buffer, _count - first);
    _tail = (_tail + _count) % (_size == 0 ? 1 : _size);
    _count = 0;
}

void DMATxRing::write_polled(const uint8_t *data, size_t length)
{
    if (_in_flight > 0) {
        // The piece with the DMA finishes on its own; what follows it goes by hand
        _port.finish();
        _tail = (_tail + _in_flight) % _size;
        _count -= _in_flight;
        _in_flight = 0;
    }
    if (_count > 0) {
        send_polled_ring();
    }
    if (_dropped_pending > 0) {
        char note[note_size];
        const size_t note_length = format_note(note, _dropped_pending);
        _port.send_polled(reinterpret_cast<const uint8_t *>(note), note_length);
        _dropped_pending = 0;
    }
    if (length > 0) {
        _port.send_polled(data, length);
    }
    if (_state != State::Idle) {
        // send_polled() waited for the line: a drain armed before has nothing left to wait for
        _state = State::Idle;
        _port.allow_deep_sleep(true);
    }
}

} // namespace internal
} // namespace mbed
