/**
 * ep-oc-mcu
 * Embedded Planet Open Core for Microcontrollers
 *
 * Built with ARM Mbed-OS
 *
 * Copyright (c) 2019-2020 Embedded Planet, Inc.
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
 *
 */

#include "MCP23008.hpp"

#include "mbed_error.h"
#include "mbed_assert.h"

namespace {
const uint8_t MCP23008_ADDRESS  = 0x40;

/* MCP23008 registers */
const uint8_t IODIR             = 0x00;
const uint8_t IPOL              = 0x01;
const uint8_t GPINTEN           = 0x02;
const uint8_t DEFVAL            = 0x03;
const uint8_t INTCON            = 0x04;
const uint8_t IOCON             = 0x05;
const uint8_t GPPU              = 0x06;
const uint8_t INTF              = 0x07;
const uint8_t INTCAP            = 0x08;
const uint8_t GPIO              = 0x09;
const uint8_t OLAT              = 0x0A;
};

MCP23008::MCP23008 ( PinName sda, PinName scl, uint8_t address, Frequency freq )
        : i2c ( sda, scl ),
        i2c_address ( MCP23008_ADDRESS | (address << 1) ) {
    if ( address > 7 )
        error ( "MCP23008::MCP23008: address is out of range, must be <= 7\n" );

    i2c.frequency ( ( int ) freq );
    reset ();
}

void MCP23008::set_input_pins ( uint8_t pins ) {
    uint8_t value = read_register ( IODIR );
    write_register ( IODIR, value | pins );
}

void MCP23008::set_output_pins ( uint8_t pins ) {
    uint8_t value = read_register ( IODIR );
    write_register ( IODIR, value & ~pins );
}

void MCP23008::write_outputs ( uint8_t values ) {
    write_register ( GPIO, values );
}

uint8_t MCP23008::read_outputs () {
    return read_register ( OLAT );
}

uint8_t MCP23008::read_inputs () {
    return read_register ( GPIO );
}

void MCP23008::set_input_polarity ( uint8_t values ) {
    write_register ( IPOL, values );
}

uint8_t MCP23008::get_input_polarity () {
    return read_register ( IPOL );
}

void MCP23008::set_pullups ( uint8_t values ) {
    write_register ( GPPU, values );
}

uint8_t MCP23008::get_pullups () {
    return read_register ( GPPU );
}

void MCP23008::interrupt_on_changes ( uint8_t pins ) {
    uint8_t value = read_register ( INTCON );
    value &= ~pins;
    write_register ( INTCON, value );
    value = read_register ( GPINTEN );
    value |= pins;
    write_register ( GPINTEN, value );
}

void MCP23008::disable_interrupts ( uint8_t pins ) {
    uint8_t value = read_register ( GPINTEN );
    value &= ~pins;
    write_register ( GPINTEN, value );
}

void MCP23008::acknowledge_interrupt ( uint8_t &pin, uint8_t &values ) {
    pin = read_register ( INTF );
    values = read_register ( INTCAP );
}

uint8_t MCP23008::read_register ( uint8_t reg ) {
    mutex.lock();
    char data[] = {reg};
    if ( 0 != i2c.write ( i2c_address, data, 1 ) )
        error ( "MCP23008::read_register: Missing ACK for write\n" );

    if ( 0 != i2c.read ( i2c_address, data, 1 ) )
        error ( "MCP23008:read_register: Missing ACK for read\n" );
    mutex.unlock();
    return data[0];
}

void MCP23008::write_register ( uint8_t reg, uint8_t value ) {
    char data[] = {reg, value};
    if ( 0 != i2c.write ( i2c_address, data, 2 ) )
        error ( "MCP23008::write_register: Missing ACK for write\n" );
}

void MCP23008::reset ( ) {
    write_register ( IODIR, 0xFF );
    for ( uint8_t reg = IPOL; reg <= OLAT; reg++ )
        write_register ( reg, 0 );
}

MCP23008::ExpandedIO::ExpandedIO(MCP23008& parent, Pin pin) : _parent(parent),
        _pin(pin) {
}

int MCP23008::ExpandedIO::read() {
    return (_parent.read_inputs() & _pin);
}

void MCP23008::ExpandedIO::mode(PinMode pull) {

    // PullDown is not supported by the MCP23008
    MBED_ASSERT(pull != PullDown);

    _parent.mutex.lock();
    uint8_t pullups = _parent.get_pullups();

    if(pull == PullNone) {
        pullups &= ~_pin;
    } else if(pull == PullUp) {
        pullups |= _pin;
    }

    _parent.set_pullups(pullups);
    _parent.mutex.unlock();
}

void MCP23008::ExpandedIO::write(int value) {
    uint8_t outputs = _parent.read_outputs();
    if(value) {
        _parent.write_outputs((outputs | _pin));
    } else {
        _parent.write_outputs((outputs & (~_pin)));
    }
}

void MCP23008::ExpandedIO::output() {
    _parent.set_output_pins(_pin);
}

void MCP23008::ExpandedIO::input() {
    _parent.set_input_pins(_pin);
}

MCP23008::ExpandedInput MCP23008::as_input(Pin pin) {
    return ExpandedInput(*this, pin);
}

MCP23008::ExpandedOutput MCP23008::as_output(Pin pin) {
    return ExpandedOutput(*this, pin);
}

MCP23008::ExpandedInputOutput MCP23008::as_input_output(Pin pin) {
    return ExpandedInputOutput(*this, pin);
}
