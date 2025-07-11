/*
    SPI library for the Raspberry Pi Pico RP2040

    Copyright (c) Mircea Neacsu 2025

    Adapted from https://github.com/earlephilhower/arduino-pico/tree/master/libraries/SPI/src
    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include <hardware/spi.h>


namespace RP2040
{

///Implements a hardware-based SPI interface using the Pico's SPI blocks
class SPI
{
public:

  enum mode_t {MODE0, MODE1, MODE2, MODE3};

  /// Create an SPI instance
  SPI (uint8_t channel, uint8_t rx, uint8_t cs, uint8_t sck, uint8_t tx, bool user_cs = false);

  /// Initializes the SPI interface
  void init (uint baud, mode_t mode=MODE0);

  /// Stops the SPI interface
  void deinit ();

  /// Return `true` if  interface is up and running
  bool is_initialized () const;

///@{

  /// Set operating mode as master or slave
  void set_master (bool ms);

  /// Sets the RX pin. Call before init()
  void RXpin (uint8_t pin);

  ///Sets the MISO(RX) pin. Call before init()
  void MISOpin (uint8_t pin);

  /// Sets the TX pin. Call before init()
  void TXpin (uint8_t pin);

  /// Sets the MOSI pin. Call before init()
  void MOSIpin (int8_t pin);

  /// Sets the CS pin. Call before init()
  void CSpin (uint8_t pin, bool usr = false);

  /// Sets the CLK pin. Call before init()
  void CLKpin (uint8_t pin);

  /// Change transfered word size
  void set_data_size (uint8_t sz);

///@}

  /// Set interface to loopback mode
  void loopback (bool on);

  /// Send an 8-bit byte of data and return read-back 8-bit value
  uint8_t transfer(uint8_t data);

  /// Send a 16-bit half-word and return the read-back 16-bit value
  uint16_t transfer16(uint16_t data);

  /// Sends `count` bytes from `src` under a single CS. If `dst` is not null, receives `count`
  void transfer(const uint8_t *src, uint8_t* dst, size_t count);

  // DMA transfers.  Do not combime with synchronous runs or bad stuff will happen
  // All buffers must be valid for entire DMA and not touched until `finished()` returns true.

  /// Perform a transfer() using DMA in the background. Returns immediately, need to check for completion
  bool transferDMA(const uint8_t *send, uint8_t *recv, size_t bytes);

  /// Call to check if the async operations is completed and the buffer can be reused/read
  bool finishedDMA();

  /**
      @brief Aborts an ongoing asynchronous SPI operation, if one is still operating

      @details
      Not normally needed, but in the case where a large, long SPI operation needs to be aborted
      this call allows an application to safely stop the SPI and dispose of the ``recv`` and
      ``send`` buffers
  */
  void abortDMA();
    
private:
  spi_inst_t *spi;

  uint baud;
  mode_t mode;
  uint8_t dlen; //data length (4 to 16)
  bool master;

  int8_t RX, TX, SCK, CS; //pins
  bool usr_cs;
  bool running; // SPI port active
  bool initted; // Transaction begun

  // DMA
  int channel_rx_DMA;
  int channel_tx_DMA;
};

// Inline functions--------------------------------------------------------------------------------

/*!
  \param ms [in] If true set interface as master

  \note Definitions of MISO and MOSI pins changes between master and slave mode
*/
inline
void SPI::set_master(bool ms) {
  assert (!initted);
  master = mode;
}

/*!
  \param [in] pin The GPIO number to assign to

  \note Definition of MOSI and MISO pins changes between master and slave modes
*/
inline
void SPI::MISOpin (uint8_t pin) {
  master?RXpin(pin) : TXpin(pin);
}

/*!
  \param [in] pin The GPIO number to assign to

  \note Definition of MOSI and MISO pins changes between master and slave modes
*/
inline
void SPI::MOSIpin(int8_t pin) {
  master?TXpin(pin) : RXpin(pin);
}

inline
bool SPI::is_initialized() const {
  return initted;
}

} //end namespace