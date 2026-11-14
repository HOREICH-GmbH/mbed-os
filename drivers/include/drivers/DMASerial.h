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
#ifndef MBED_DMA_SERIAL_H
#define MBED_DMA_SERIAL_H

#include "platform/platform.h"

#if DEVICE_SERIAL_DMA || defined(DOXYGEN_ONLY)

#include "drivers/SerialBase.h"
#include "drivers/internal/DMARxRing.h"
#include "drivers/internal/DMATxRing.h"
#include "hal/serial_dma_api.h"
#include "platform/FileHandle.h"
#include "platform/NonCopyable.h"
#include "rtos/Mutex.h"
#if MBED_CONF_RTOS_PRESENT
#include "rtos/EventFlags.h"
#endif

namespace mbed {
/**
 * \defgroup drivers_DMASerial DMASerial class
 * \ingroup drivers-public-api-uart
 * @{
 */

/** A UART by DMA both ways, as a FileHandle - BufferedSerial's interface, with one interrupt per burst or piece
 *  instead of one per byte, and reception that goes on while the CPU stalls (a flash erase).
 *
 *  Transmit: write() copies into the caller's ring, which the DMA sends from a contiguous piece at a time. When it is
 *  full, WhenFull::Drop drops the write and counts it ("[n bytes dropped]" goes out where the bytes were lost - a
 *  console must hold nobody up); WhenFull::Wait takes what fits and waits for room for the rest (a modem: no byte of a
 *  command may go missing; non-blocking, the rest is the caller's - poll() says POLLOUT when there is room).
 *  Receive (optional): a circular DMA into the caller's ring; the line going idle after a burst and the DMA passing
 *  the ring's middle and end wake a reader and call sigio(). Bytes the DMA overwrote before they were read are
 *  counted (rx_overruns()), line errors too (errors()), and reception goes on.
 *
 *  The UART is mbed's serial_api's (SerialBase); the DMA is hal/serial_dma_api.h's. When the DMA cannot be had - no
 *  link known for this UART, or a channel another user's - the constructor gives the UART back (is_dma() is false)
 *  and the caller falls back, e.g. to a BufferedSerial on the same pins.
 *
 *  Power: enable_input(false) and enable_output(false) both - the DMA stopped, the UART released (serial_free():
 *  reset, clock off, pins analog - what deep sleep needs of it), deep sleep allowed; either on again sets it all up.
 *  Deep sleep is held off while receiving, and while a transmission is under way.
 *
 *  With interrupts off (mbed's fatal error report) a write waits for the running transfer and sends the rest polled.
 */
class DMASerial:
    private SerialBase,
    public FileHandle,
    private internal::DMATxRing::Port,
    private NonCopyable<DMASerial> {

public:
    enum class WhenFull {
        Drop, ///< the write dropped and counted - never waits
        Wait, ///< as much as fits taken, the rest waits for room
    };

    /** A UART by DMA
     *
     *  @param tx        Transmit pin
     *  @param rx        Receive pin, NC: transmit only
     *  @param baud      Baud rate
     *  @param tx_buffer The transmit ring, the caller's for the DMASerial's life
     *  @param tx_size   Its size (up to 65535)
     *  @param rx_buffer The receive ring; nullptr: transmit only
     *  @param rx_size   Its size (up to 65535)
     *  @param when_full What a write does when the transmit ring is full
     */
    DMASerial(PinName tx, PinName rx, int baud, uint8_t *tx_buffer, size_t tx_size, uint8_t *rx_buffer = nullptr,
              size_t rx_size = 0, WhenFull when_full = WhenFull::Wait);

    ~DMASerial() override;

    /** The DMA attached; false: the UART was given back and nothing goes through this object */
    bool is_dma() const
    {
        return _dma_ready;
    }

    /** Bytes dropped by a full transmit ring (WhenFull::Drop) */
    uint32_t tx_dropped() const;

    /** Bytes the receive DMA overwrote before they were read */
    uint32_t rx_overruns() const;

    /** Line errors (framing, noise, parity) and DMA transfer errors */
    uint32_t errors() const
    {
        return _errors;
    }

    /** Set the baud rate */
    void set_baud(int baud);

    using SerialBase::Parity;
    using SerialBase::None;
    using SerialBase::Odd;
    using SerialBase::Even;
    using SerialBase::Forced1;
    using SerialBase::Forced0;

    /** Set the transmission format used by the serial port - kept, and set again at every power-up (SerialBase's
     *  set-up restores the rate and flow control only) */
    void set_format(int bits = 8, Parity parity = DMASerial::None, int stop_bits = 1);

#if DEVICE_SERIAL_FC
    using SerialBase::Flow;
    using SerialBase::Disabled;
    using SerialBase::RTS;
    using SerialBase::CTS;
    using SerialBase::RTSCTS;

    /** Set the flow control type on the serial port */
    void set_flow_control(Flow type, PinName flow1 = NC, PinName flow2 = NC);
#endif

    ssize_t write(const void *buffer, size_t length) override;
    ssize_t read(void *buffer, size_t length) override;
    int close() override;
    int isatty() override;
    off_t seek(off_t offset, int whence) override;
    off_t size() override;
    /** Waits (up to a second) until everything has left the line; polled with interrupts off */
    int sync() override;
    int set_blocking(bool blocking) override;
    bool is_blocking() const override
    {
        return _blocking;
    }
    int enable_input(bool enabled) override;
    int enable_output(bool enabled) override;
    short poll(short events) const override;
    void sigio(Callback<void()> func) override;

private:
    // internal::DMATxRing::Port - with interrupts off
    void start(const uint8_t *data, size_t length) override;
    void finish() override;
    void send_polled(const uint8_t *data, size_t length) override;
    void notify_idle() override;
    void allow_deep_sleep(bool allow) override;

    // SerialBase
    void lock() override;
    void unlock() override;

    static void on_dma(void *context, uint32_t events);

    bool receives() const
    {
        return _rx_ring.size() > 0;
    }
    void power_up();
    void power_down();
    void reconcile();        ///< powered and receiving as enable_input()/enable_output() ask
    void start_receiving();
    void stop_receiving();
    void update_received() const; ///< the ring from where the DMA is - interrupts off
    void readable();         ///< bytes arrived: sigio, a blocked read woken - interrupt context
    void wait_for(uint32_t flag);
    bool kernel_running() const;

    static constexpr uint32_t readable_flag = 1u << 0;
    static constexpr uint32_t room_flag = 1u << 1;

    serial_dma_t _dma {};
    internal::DMATxRing _tx_ring;
    mutable internal::DMARxRing _rx_ring; ///< mutable: poll() is const, and asks where the DMA is
#if MBED_CONF_RTOS_PRESENT
    rtos::EventFlags _flags;
#endif
    rtos::Mutex _mutex;
    Callback<void()> _sigio;
    const WhenFull _when_full;
    volatile uint32_t _errors = 0;
    bool _dma_ready = false;
    bool _powered = false;
    bool _receiving = false;
    bool _input_enabled = true;
    bool _output_enabled = true;
    bool _blocking = true;
    bool _tx_deep_sleep_held = false;
    // The format set_format() asked for: SerialBase keeps none, and its _init() leaves the UART at 8N1
    int _format_bits = 8;
    Parity _format_parity = None;
    int _format_stop_bits = 1;
    bool _rx_deep_sleep_held = false;
};

/** @}*/

} // namespace mbed

#endif // DEVICE_SERIAL_DMA || defined(DOXYGEN_ONLY)

#endif // MBED_DMA_SERIAL_H
