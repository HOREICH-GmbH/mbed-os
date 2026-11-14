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
#include "drivers/DMASerial.h"

#if DEVICE_SERIAL_DMA

#include "platform/mbed_critical.h"
#include "platform/mbed_poll.h"
#include "platform/mbed_power_mgmt.h"
#include "platform/mbed_thread.h"
#include "platform/mbed_wait_api.h"
#if MBED_CONF_RTOS_PRESENT
#include "cmsis_os2.h"
#endif

#include <errno.h>
#include <stdio.h>
#if MBED_CONF_RTOS_PRESENT
#include "rtos/Kernel.h"
#endif

// No trace here: the console may be one of these.

namespace mbed {

namespace {
constexpr size_t max_transfer = 0xFFFF; // a DMA block's length
}

DMASerial::DMASerial(PinName tx, PinName rx, int baud, uint8_t *tx_buffer, size_t tx_size, uint8_t *rx_buffer,
                     size_t rx_size, WhenFull when_full) :
    SerialBase(tx, rx, baud),
    _tx_ring(*this, tx_buffer, tx_size <= max_transfer ? tx_size : 0),
    _rx_ring(rx != NC ? rx_buffer : nullptr, (rx != NC && rx_size <= max_transfer) ? rx_size : 0),
    _when_full(when_full)
{
    // SerialBase has set the UART up (serial_init, serial_baud); the DMA on top of it
    _dma_ready = tx_buffer != nullptr && tx_size > 0 && tx_size <= max_transfer &&
                 serial_dma_attach(&_dma, &_serial, true, receives() ? rx_buffer : nullptr, _rx_ring.size(),
                                   &DMASerial::on_dma, this) == 0;
    if (!_dma_ready) {
        // Given back, for a fallback on the same pins
        _deinit();
        _rx_enabled = false;
        _tx_enabled = false;
        return;
    }
    _powered = true;
    reconcile(); // receiving, as a BufferedSerial is from its construction
}

DMASerial::~DMASerial()
{
    if (!_dma_ready) {
        return;
    }
    if (_powered) {
        sync(); // what is under way leaves the line first (bounded)
    }
    serial_dma_detach(&_dma);
    allow_deep_sleep(true);
    if (_rx_deep_sleep_held) {
        sleep_manager_unlock_deep_sleep();
        _rx_deep_sleep_held = false;
    }
    if (_powered) {
        _deinit();
        _rx_enabled = false;
        _tx_enabled = false; // SerialBase's destructor: nothing left to free
    }
}

bool DMASerial::kernel_running() const
{
#if MBED_CONF_RTOS_PRESENT
    return osKernelGetState() == osKernelRunning && !core_util_is_isr_active();
#else
    return !core_util_is_isr_active();
#endif
}

void DMASerial::lock()
{
    if (kernel_running()) {
        _mutex.lock();
    }
}

void DMASerial::unlock()
{
    if (kernel_running()) {
        _mutex.unlock();
    }
}

void DMASerial::wait_for(uint32_t flag)
{
#if MBED_CONF_RTOS_PRESENT
    // Bounded: a piece done or bytes in set the flag, but a stuck line must not hold the caller for ever
    _flags.wait_any_for(flag, rtos::Kernel::Clock::duration_u32(100));
#else
    (void)flag;
    thread_sleep_for(1);
#endif
}

// ---- Power and the UART ------------------------------------------------------------------------------------------

void DMASerial::reconcile()
{
    const bool want_power = _input_enabled || _output_enabled;
    if (want_power && !_powered) {
        power_up();
    } else if (!want_power && _powered) {
        power_down();
        return;
    }
    if (_powered && receives()) {
        if (_input_enabled && !_receiving) {
            start_receiving();
        } else if (!_input_enabled && _receiving) {
            stop_receiving();
        }
    }
}

void DMASerial::power_up()
{
    // mbed's serial layer (pins, clock, HAL init, the rate and flow control SerialBase keeps), then the DMA on top
    (this->*_init_func)();
    if (_format_bits != 8 || _format_parity != None || _format_stop_bits != 1) {
        SerialBase::format(_format_bits, _format_parity, _format_stop_bits); // _init() left it at 8N1
    }
    _rx_enabled = true;
    _tx_enabled = true;
    serial_dma_reapply(&_dma);
    _powered = true;
    if (_frame != Frame::None) {
        apply_frame(); // set while powered down, or a gap from a rate set meanwhile
    }
}

void DMASerial::power_down()
{
    sync(); // what is under way leaves the line first (bounded)
    core_util_critical_section_enter();
    serial_dma_tx_stop(&_dma);
    _tx_ring.reset();
    core_util_critical_section_exit();
    if (_receiving) {
        stop_receiving();
    }
    serial_dma_suspend(&_dma);
    _deinit(); // reset, clock off, pins released - what deep sleep needs of a UART
    _rx_enabled = false;
    _tx_enabled = false;
    _powered = false;
}

void DMASerial::start_receiving()
{
    core_util_critical_section_enter();
    _rx_ring.reset();
    core_util_critical_section_exit();
    serial_dma_rx_start(&_dma);
    if (!_rx_deep_sleep_held) {
        sleep_manager_lock_deep_sleep(); // the UART does not receive in deep sleep
        _rx_deep_sleep_held = true;
    }
    _receiving = true;
}

void DMASerial::stop_receiving()
{
    serial_dma_rx_stop(&_dma);
    _receiving = false;
    if (_rx_deep_sleep_held) {
        sleep_manager_unlock_deep_sleep();
        _rx_deep_sleep_held = false;
    }
}

void DMASerial::update_received() const
{
    if (!_receiving) {
        return;
    }
    uint32_t laps = 0;
    const uint32_t position = serial_dma_rx_position(const_cast<serial_dma_t *>(&_dma), &laps);
    _rx_ring.update(laps, position);
}

// ---- Settings ----------------------------------------------------------------------------------------------------

uint32_t DMASerial::tx_dropped() const
{
    return _tx_ring.dropped();
}

uint32_t DMASerial::check_health() const
{
    if (!_dma_ready || !_powered) {
        return 0;
    }
    return serial_dma_check(const_cast<serial_dma_t *>(&_dma));
}

uint32_t DMASerial::stray_interrupts() const
{
    return _dma_ready ? serial_dma_stray_interrupts(const_cast<serial_dma_t *>(&_dma)) : 0;
}

int DMASerial::describe(char *out, size_t size) const
{
    if (!_dma_ready) {
        return snprintf(out, size, "no DMA");
    }
    core_util_critical_section_enter();
    update_received();
    const size_t available = _rx_ring.available();
    const size_t frames = _rx_ring.frames();
    const uint32_t overruns = _rx_ring.overruns();
    const uint32_t merged = _rx_ring.frames_merged();
    core_util_critical_section_exit();
    const int head = snprintf(out, size, "%s rx %u frames %u overruns %lu errors %lu merged %lu cut %lu dropped %lu faults %lx | ",
                              _powered ? "on" : "off", (unsigned)available, (unsigned)frames, (unsigned long)overruns,
                              (unsigned long)_errors, (unsigned long)merged, (unsigned long)_frames_truncated,
                              (unsigned long)tx_dropped(), (unsigned long)check_health());
    if (head < 0 || (size_t)head >= size) {
        return head;
    }
    return head + serial_dma_describe(const_cast<serial_dma_t *>(&_dma), out + head, size - (size_t)head);
}

uint32_t DMASerial::frames_merged() const
{
    core_util_critical_section_enter();
    const uint32_t merged = _rx_ring.frames_merged();
    core_util_critical_section_exit();
    return merged;
}

uint32_t DMASerial::modbus_frame_gap_bits(int baud, int char_bits)
{
    if (baud > 19200) {
        return (uint32_t)((1750ULL * (uint64_t)baud + 999999ULL) / 1000000ULL); // 1750 us, rounded up
    }
    return (uint32_t)((7 * char_bits + 1) / 2); // 3.5 characters, rounded up
}

int DMASerial::char_bits() const
{
    return 1 + _format_bits + (_format_parity != None ? 1 : 0) + _format_stop_bits;
}

int DMASerial::apply_frame()
{
    serial_dma_frame_t mode = SERIAL_DMA_FRAME_NONE;
    uint32_t value = _frame_value;
    switch (_frame) {
        case Frame::Idle:
            mode = SERIAL_DMA_FRAME_IDLE;
            break;
        case Frame::Timeout:
            mode = SERIAL_DMA_FRAME_TIMEOUT;
            if (_frame_gap_auto) {
                value = modbus_frame_gap_bits(_baud, char_bits());
            }
            break;
        case Frame::Match:
            mode = SERIAL_DMA_FRAME_MATCH;
            break;
        default:
            break;
    }
    const int rc = serial_dma_rx_frame(&_dma, mode, value);
    return rc == 0 ? 0 : (rc == -1 ? -ENOTSUP : -EINVAL);
}

int DMASerial::set_frame(Frame frame, uint32_t value)
{
    if (!_dma_ready || !receives()) {
        return -EINVAL;
    }
    lock();
    const Frame previous = _frame;
    const uint32_t previous_value = _frame_value;
    const bool previous_auto = _frame_gap_auto;
    _frame = frame;
    _frame_value = value;
    _frame_gap_auto = frame == Frame::Timeout && value == 0;
    int rc = 0;
    if (_powered) {
        rc = apply_frame();
        if (rc != 0) {
            // Refused: the setting before stays
            _frame = previous;
            _frame_value = previous_value;
            _frame_gap_auto = previous_auto;
        }
    }
    unlock();
    return rc;
}

ssize_t DMASerial::read_frame(void *buffer, size_t length, std::chrono::milliseconds timeout)
{
    if (!receives() || !_dma_ready || _frame == Frame::None) {
        return -EINVAL;
    }
#if MBED_CONF_RTOS_PRESENT
    const auto deadline = rtos::Kernel::Clock::now() + timeout;
#else
    auto left = timeout;
#endif
    for (;;) {
#if MBED_CONF_RTOS_PRESENT
        _flags.clear(readable_flag);
#endif
        bool truncated = false;
        core_util_critical_section_enter();
        update_received();
        const size_t count = _rx_ring.read_frame(static_cast<uint8_t *>(buffer), length, &truncated);
        core_util_critical_section_exit();
        if (count > 0) {
            if (truncated) {
                _frames_truncated = _frames_truncated + 1;
            }
            return static_cast<ssize_t>(count);
        }
        if (!kernel_running()) {
            return 0; // no waiting in an interrupt or before the kernel
        }
#if MBED_CONF_RTOS_PRESENT
        const auto now = rtos::Kernel::Clock::now();
        if (now >= deadline) {
            return 0;
        }
        _flags.wait_any_for(readable_flag, std::chrono::duration_cast<rtos::Kernel::Clock::duration_u32>(deadline - now));
#else
        if (left <= std::chrono::milliseconds(0)) {
            return 0;
        }
        thread_sleep_for(1);
        left -= std::chrono::milliseconds(1);
#endif
    }
}

uint32_t DMASerial::rx_overruns() const
{
    core_util_critical_section_enter();
    update_received();
    const uint32_t overruns = _rx_ring.overruns();
    core_util_critical_section_exit();
    return overruns;
}

void DMASerial::set_baud(int baud)
{
    lock();
    if (_powered) {
        SerialBase::baud(baud); // HAL_UART_Init anew
        if (_dma_ready) {
            serial_dma_reapply(&_dma);
            if (_frame_gap_auto) {
                apply_frame(); // Modbus RTU's gap for the new rate
            }
        }
    } else {
        _baud = baud; // applied at power-up
    }
    unlock();
}

void DMASerial::set_format(int bits, Parity parity, int stop_bits)
{
    lock();
    // Kept: powered down it is applied at power-up, and every power-up sets it again
    _format_bits = bits;
    _format_parity = parity;
    _format_stop_bits = stop_bits;
    if (_powered) {
        SerialBase::format(bits, parity, stop_bits);
        if (_dma_ready) {
            serial_dma_reapply(&_dma);
            if (_frame_gap_auto) {
                apply_frame(); // Modbus RTU's gap for the new character length
            }
        }
    }
    unlock();
}

#if DEVICE_SERIAL_FC
void DMASerial::set_flow_control(Flow type, PinName flow1, PinName flow2)
{
    lock();
    if (_powered) {
        SerialBase::set_flow_control(type, flow1, flow2);
        if (_dma_ready) {
            serial_dma_reapply(&_dma);
        }
    } else {
        // Kept for SerialBase's _init() at power-up
        _set_flow_control_dp_func = &SerialBase::set_flow_control;
        _flow_type = type;
        _flow1 = flow1;
        _flow2 = flow2;
    }
    unlock();
}
#endif

// ---- FileHandle --------------------------------------------------------------------------------------------------

ssize_t DMASerial::write(const void *buffer, size_t length)
{
    if (!_dma_ready) {
        return static_cast<ssize_t>(length); // nowhere to go: gone
    }
    const uint8_t *data = static_cast<const uint8_t *>(buffer);
    if (!core_util_are_interrupts_enabled()) {
        // Nothing will interrupt to move the ring on: now, by hand (a fatal error, mbed_error_puts())
        if (_powered) {
            _tx_ring.write_polled(data, length);
        }
        return static_cast<ssize_t>(length);
    }
    if (_when_full == WhenFull::Drop) {
        if (_powered && _output_enabled) {
            core_util_critical_section_enter();
            _tx_ring.write(data, length);
            core_util_critical_section_exit();
        }
        return static_cast<ssize_t>(length);
    }
    // WhenFull::Wait: as much as fits, then room for the rest - or, non-blocking, what was taken. One writer at a time,
    // so a command is not interleaved with another thread's.
    lock();
    size_t done = 0;
    while (done < length && _powered && _output_enabled) {
#if MBED_CONF_RTOS_PRESENT
        _flags.clear(room_flag);
#endif
        core_util_critical_section_enter();
        done += _tx_ring.write_some(data + done, length - done);
        core_util_critical_section_exit();
        if (done == length || !_blocking || !kernel_running()) {
            break;
        }
        wait_for(room_flag); // a piece done makes room
    }
    unlock();
    if (done == 0 && length > 0) {
        return -EAGAIN;
    }
    return static_cast<ssize_t>(done);
}

ssize_t DMASerial::read(void *buffer, size_t length)
{
    if (!receives() || !_dma_ready) {
        return 0; // transmit only
    }
    for (;;) {
#if MBED_CONF_RTOS_PRESENT
        _flags.clear(readable_flag);
#endif
        core_util_critical_section_enter();
        update_received();
        const size_t count = _rx_ring.read(static_cast<uint8_t *>(buffer), length);
        core_util_critical_section_exit();
        if (count > 0 || length == 0) {
            return static_cast<ssize_t>(count);
        }
        if (!_blocking || !kernel_running()) {
            return -EAGAIN;
        }
#if MBED_CONF_RTOS_PRESENT
        _flags.wait_any(readable_flag);
#else
        thread_sleep_for(1);
#endif
    }
}

int DMASerial::close()
{
    return 0;
}

int DMASerial::isatty()
{
    return 1; // a terminal: mbed's stdio turns "\n" into "\r\n" for it
}

off_t DMASerial::seek(off_t offset, int whence)
{
    (void)offset;
    (void)whence;
    return -ESPIPE;
}

off_t DMASerial::size()
{
    return -EINVAL;
}

int DMASerial::sync()
{
    if (!_dma_ready || !_powered) {
        return 0;
    }
    if (!core_util_are_interrupts_enabled()) {
        _tx_ring.write_polled(nullptr, 0);
        return 0;
    }
    // A full 2 KB ring takes ~22 ms at 921600 baud, ~180 ms at 115200; give up after a second
    for (int ms = 0; ms < 1000 && _tx_ring.busy(); ms++) {
        if (kernel_running()) {
            thread_sleep_for(1);
        } else {
            wait_us(1000);
        }
    }
    return _tx_ring.busy() ? -EAGAIN : 0;
}

int DMASerial::set_blocking(bool blocking)
{
    _blocking = blocking;
    return 0;
}

int DMASerial::enable_input(bool enabled)
{
    lock();
    _input_enabled = enabled;
    if (_dma_ready) {
        reconcile();
    }
    unlock();
    return 0;
}

int DMASerial::enable_output(bool enabled)
{
    lock();
    _output_enabled = enabled;
    if (_dma_ready) {
        reconcile();
    }
    unlock();
    return 0;
}

short DMASerial::poll(short events) const
{
    short ready = 0;
    if (receives() && _dma_ready) {
        core_util_critical_section_enter();
        update_received();
        const bool readable = _rx_ring.available() > 0;
        core_util_critical_section_exit();
        ready |= readable ? POLLIN : 0;
    }
    if (_dma_ready && _powered && _output_enabled && (_when_full == WhenFull::Drop || _tx_ring.free() > 0)) {
        ready |= POLLOUT;
    }
    return ready & events;
}

void DMASerial::sigio(Callback<void()> func)
{
    core_util_critical_section_enter();
    _sigio = func;
    core_util_critical_section_exit();
    // As BufferedSerial: bytes already there are said at once
    if (func && (poll(POLLIN) & POLLIN)) {
        func();
    }
}

// ---- internal::DMATxRing::Port, interrupts off -------------------------------------------------------------------

void DMASerial::start(const uint8_t *data, size_t length)
{
    if (serial_dma_tx_start(&_dma, data, static_cast<uint32_t>(length)) != 0) {
        // Out by hand, then on as if the DMA had done it
        _errors = _errors + 1;
        serial_dma_tx_polled(&_dma, data, static_cast<uint32_t>(length));
        _tx_ring.completed();
    }
}

void DMASerial::finish()
{
    serial_dma_tx_finish(&_dma);
}

void DMASerial::send_polled(const uint8_t *data, size_t length)
{
    serial_dma_tx_polled(&_dma, data, static_cast<uint32_t>(length));
}

void DMASerial::notify_idle()
{
    serial_dma_tx_notify_idle(&_dma);
}

void DMASerial::allow_deep_sleep(bool allow)
{
    if (!allow && !_tx_deep_sleep_held) {
        sleep_manager_lock_deep_sleep();
        _tx_deep_sleep_held = true;
    } else if (allow && _tx_deep_sleep_held) {
        sleep_manager_unlock_deep_sleep();
        _tx_deep_sleep_held = false;
    }
}

// ---- Interrupts --------------------------------------------------------------------------------------------------

void DMASerial::readable()
{
#if MBED_CONF_RTOS_PRESENT
    _flags.set(readable_flag);
#endif
    if (_sigio) {
        _sigio();
    }
}

void DMASerial::on_dma(void *context, uint32_t events)
{
    DMASerial *self = static_cast<DMASerial *>(context);
    if (events & SERIAL_DMA_ERROR) {
        self->_errors = self->_errors + 1;
    }
    if (events & (SERIAL_DMA_TX_CONSUMED | SERIAL_DMA_TX_IDLE)) {
        core_util_critical_section_enter(); // the UART's interrupt, the DMA's and the writers share the ring
        if (events & SERIAL_DMA_TX_CONSUMED) {
            self->_tx_ring.completed();
        }
        if (events & SERIAL_DMA_TX_IDLE) {
            self->_tx_ring.idle();
        }
        core_util_critical_section_exit();
#if MBED_CONF_RTOS_PRESENT
        if (events & SERIAL_DMA_TX_CONSUMED) {
            self->_flags.set(room_flag);
        }
#endif
    }
    if (events & SERIAL_DMA_RX_RESTART) {
        core_util_critical_section_enter();
        self->_rx_ring.reset();
        core_util_critical_section_exit();
    }
    if (events & SERIAL_DMA_RX_FRAME) {
        // Where the DMA was as the frame ended - captured in the UART's interrupt, this one
        uint32_t laps = 0;
        const uint32_t position = serial_dma_rx_frame_end(&self->_dma, &laps);
        core_util_critical_section_enter();
        self->_rx_ring.frame_end(laps, position);
        core_util_critical_section_exit();
    }
    if (events & (SERIAL_DMA_RX | SERIAL_DMA_RX_RESTART)) {
        self->readable();
    }
}

} // namespace mbed

#endif // DEVICE_SERIAL_DMA
