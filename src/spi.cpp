/*
    SPI library for the Raspberry Pi Pico RP2040

    Copyright (c) Mircea Neacsu 2025
    Transformed the Arduino libary into a stand-alone libary for Rasperry Pi Pico

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

//#define _DEBUG

#ifdef _DEBUG
#define PARAM_ASSERTIONS_ENABLED_SPI 1
#endif

#include <rpp/spi.h>

#include <hardware/dma.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/structs/iobank0.h>
#include <hardware/irq.h>
#include <hardware/address_mapped.h>
#include <memory.h>

#ifdef _DEBUG
#include <stdio.h>
#endif

#ifdef _DEBUG
#define DEBUGSPI(...) printf(__VA_ARGS__)
#else
#define DEBUGSPI(...)
#endif


namespace RP2040
{

//valid RX pins
static constexpr uint64_t valid_rx [2] = { 
  0x110011, // bits 0, 4, 16, 20 for SPI0
  0x11001100 // bits 8, 12, 24, 28 for SPI1
};

//valid CS pins
static constexpr uint64_t valid_cs[2] = {
  0x220022, // bits 1, 5, 17, 21 for SPI0
  0x22002200 //bits 9, 13, 25, 29 for SPI1
};

//valid CLK pins
static constexpr uint64_t valid_ck[2] = { 
  0x440044, // bits 2, 6, 18, 22 for SPI0
  0x4004400 // bits 10, 14, 26 for SPI1
};

//valid TX pins
static constexpr uint64_t valid_tx[2] = {
  0x880088, //bits 3, 7, 19, 23 for SPI0
  0x8008800 //bits 11, 15, 27 for SPI1
};


/*!
  Pins can be changed before init() call

  \param [in] channel SPI channel (0 or 1)
  \param [in] rx MISO GPIO
  \param [in] cs CS GPIO
  \param [in] sck SCK GPIO
  \param [in] tx MOSI GPIO
  \param [in] user_cs true if CS is user controlled

  All other parameters (speed, data length, mode, etc.) are set to reasonable values
*/
SPI::SPI(uint8_t channel, uint8_t rx, uint8_t cs, uint8_t sck, uint8_t tx, bool user_cs) 
  : spi (channel == 0? spi0 : spi1)
  , initted (false)
  , running (false)
  , RX (rx)
  , TX (tx)
  , SCK (sck)
  , CS (cs)
  , usr_cs (user_cs)
  , mode (MODE0)
  , baud (1000'000)
  , dlen (8)
  , master (true)
  , channel_rx_DMA (-1)
  , channel_tx_DMA (-1)
{
  invalid_params_if(HARDWARE_SPI, !((1ll << RX) & valid_rx[channel]) );
  invalid_params_if(HARDWARE_SPI, !((1ll << TX) & valid_tx[channel]) );
  invalid_params_if(HARDWARE_SPI, !((1ll << SCK) & valid_ck[channel]) );
  invalid_params_if(HARDWARE_SPI, !usr_cs && !((1ll << CS) & valid_cs[channel]) );
}

/*!
  Connect assigned GPIO pins, set format and enable the interface.

  @param [in] baud transfer speed
  @param [in] mode transfer mode
*/
void SPI::init (uint baud_, mode_t mode_)
{
  DEBUGSPI("SPI::init(%d, %d) rx=%d, cs=%d, sck=%d, tx=%d\n", baud_, mode_, RX, CS, SCK, TX);

  mode = mode_;
  baud = spi_set_baudrate (spi, baud_);
  DEBUGSPI ("SPI::init effective baudrate is %d\n", baud);

  spi_set_format(spi, dlen, 
    (mode == MODE0 || mode == MODE1)? SPI_CPOL_0 : SPI_CPOL_1, 
    (mode == MODE0 || mode == MODE2)? SPI_CPHA_0 : SPI_CPHA_1, 
    SPI_MSB_FIRST);

  // Always enable DREQ signals -- harmless if DMA is not listening
  // hw_set_bits(&spi_get_hw(spi)->dmacr, SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS);

  // Connect pins
  gpio_set_function(RX, GPIO_FUNC_SPI);
  if (!usr_cs)
    gpio_set_function(CS, GPIO_FUNC_SPI);
  
  gpio_set_function(SCK, GPIO_FUNC_SPI);
  gpio_set_function(TX, GPIO_FUNC_SPI);

  spi_set_slave (spi, !master);
    
  // Finally enable the SPI
  hw_set_bits(&spi_get_hw(spi)->cr1, SPI_SSPCR1_SSE_BITS);

  initted = true;
  DEBUGSPI ("SPI::init - done\n");
}

void SPI::deinit ()
{
  spi_deinit (spi);
  gpio_set_function(RX, GPIO_FUNC_SIO);
  if (!usr_cs)
    gpio_set_function(CS, GPIO_FUNC_SIO);
  gpio_set_function(SCK, GPIO_FUNC_SIO);
  gpio_set_function(TX, GPIO_FUNC_SIO);
  initted = false;
}

/*!
  \param [in] pin The GPIO number to assign to RX signal

  Pin number is validated against the allowed GPIO pins
*/
void SPI::RXpin (uint8_t pin)
{
  assert (!initted);
  invalid_params_if(HARDWARE_SPI, !((1ll << pin) & valid_rx[spi_get_index(spi)]) );
  RX = pin;
}

/*!
  \param [in] pin The GPIO number to assign to TX signal
*/
void SPI::TXpin (uint8_t pin) {
  assert (!initted);
  invalid_params_if(HARDWARE_SPI, !((1ll << pin) & valid_tx[spi_get_index(spi)]) );

  TX = pin;
}

/**
  @param [in] pin The GPIO number to assign to CS signal
  @param [i] user `true` if using user controlled CS

  If CS is not software controlled, pin number is validated against the allowed GPIO pins
*/
void SPI::CSpin (uint8_t pin, bool user)
{
  assert (!initted);
  usr_cs = user;
  
  if (!usr_cs)
    invalid_params_if(HARDWARE_SPI, !((1ll << pin) & valid_cs[spi_get_index(spi)]) );
  CS = pin;
}

/*!
  \param [in] pin The GPIO number to assign to clock signal
*/
void SPI::CLKpin (uint8_t pin)
{
  assert (!initted);
  invalid_params_if(HARDWARE_SPI, !((1ll << pin) & valid_ck[spi_get_index(spi)]) );
  
  SCK = pin;
}

/*!
  \param sz [in] new data size (between 4 and 16)
*/
void SPI::set_data_size (uint8_t sz)
{
  assert (4 <= sz && sz <= 16);
  if (sz == dlen)
    return; //nothing to do

  dlen = sz;
  if (!initted)
    return; // if not running nothing else to do

  //wait for any inflight transfers
  while (spi_is_busy(spi))
    tight_loop_contents ();

  hw_write_masked(&spi_get_hw(spi)->cr0, dlen - 1, SPI_SSPCR0_DSS_BITS); // Fast set data length
}

/// \param [in] on If `true`, turn on loopback mode
void SPI::loopback (bool on)
{
  hw_write_masked(&spi_get_hw(spi)->cr1, (on? 1: 0), SPI_SSPCR1_LBM_BITS); //change LBM bit
}

/*!
  \param [in] data Data to send
  \return Read back byte from SPI interface
*/
uint8_t SPI::transfer (uint8_t data)
{
  assert (initted);
  uint8_t ret;

  set_data_size (8);
  spi_write_read_blocking(spi, &data, &ret, 1);
  DEBUGSPI("SPI: read back %02x\n", ret);
  return ret;
}

/*!
    \param [in] data Data to send
    \return Read back 16-bit half-word
*/
uint16_t SPI::transfer16 (uint16_t data)
{
  assert (initted);
  uint16_t ret;

  set_data_size (16);
  spi_write16_read16_blocking(spi, &data, &ret, 1);
  DEBUGSPI("SPI: read back %04x\n", ret);
  return ret;
}

/*!
    \param [in] src Pointer to data to send
    \param [in] dst Pointer to received data
    \param [in] count Number of bytes to send/receive

    If `dst` is null any received data is discarded.
    If `src` is null, the function only receives data
*/
void SPI::transfer (const uint8_t* src, uint8_t* dst, size_t count)
{
  assert (initted);

  DEBUGSPI("SPI::transfer(%p, %p, %d)\n", src, dst, count);
  if (src && dst)
    spi_write_read_blocking (spi, src, dst, count);
  else if (src)
    spi_write_blocking (spi, src, count);
  else if (dst)
    spi_read_blocking(spi, 0xFF, dst, count);
}

/*!
  @param [in] send Buffer to transmit. If NULL this is only a receive opearation
  @param [out] recv Receiving buffer. If NULL, this is only a transmit operation
  @param [in] bytes Number of bytes to transfer

  Do not combine asynchronous and synchronous transfers.  All buffers must be valid until
  the transfer reports that it is completed i.e. finishedDMA() returns `true`.
*/
bool SPI::transferDMA(const uint8_t *send, uint8_t *recv, size_t bytes)
{
  assert (master && initted);

  DEBUGSPI("SPI::transferDMA(%p, %p, %d)\n", send, recv, bytes);

  if (!initted || (!send && !recv))
    return false;

  uint dmacr = 0;
  if (recv)
  {
    channel_rx_DMA = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(channel_rx_DMA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);      // 8b transfers into SPI FIFO
    channel_config_set_read_increment(&c, false);               // Reading same FIFO address
    channel_config_set_write_increment(&c, true);               // Writing to the buffer
    channel_config_set_dreq(&c, spi_get_dreq(spi, false));      // Wait for the RX FIFO specified
    channel_config_set_chain_to(&c, channel_rx_DMA);            // No chaining
    channel_config_set_irq_quiet(&c, true);                     // No need for IRQ
    dma_channel_configure(channel_rx_DMA, &c, recv, &spi_get_hw(spi)->dr, bytes, false);
    dmacr |= 1; //RDMAE
  }
  if (send)
  {
    channel_tx_DMA = dma_claim_unused_channel(true);
  
    dma_channel_config c = dma_channel_get_default_config(channel_tx_DMA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);      // 8b transfers into SPI FIFO
    channel_config_set_read_increment(&c, true);                // Reading increments address
    channel_config_set_write_increment(&c, false);              // Writing to the same FIFO address
    channel_config_set_dreq(&c, spi_get_dreq(spi, true));       // Wait for the TX FIFO specified
    channel_config_set_chain_to(&c, channel_tx_DMA);            // No chaining
    channel_config_set_irq_quiet(&c, true);                     // No need for IRQ
    dma_channel_configure(channel_tx_DMA, &c, &spi_get_hw(spi)->dr, send, bytes, false);
    dmacr |= 2; //TDMAE
  }

  set_data_size(8); // Set to 8-bits

  spi_get_hw(spi)->dmacr = dmacr; // TDMAE | RDMAE

  dma_channel_start(channel_rx_DMA);
  dma_channel_start(channel_tx_DMA);
  return true;
}

/*!
  \return True if the DMA operation has finished
*/

bool SPI::finishedDMA()
{
  assert (master && initted);

  if ((channel_rx_DMA > 0 && dma_channel_is_busy(channel_rx_DMA))
   || (channel_tx_DMA > 0 && dma_channel_is_busy(channel_tx_DMA))
   || (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS))
    return false;

  abortDMA (); //do cleanup
  return true;
}

void SPI::abortDMA()
{
  assert (initted);

  if (channel_rx_DMA > 0)
  {
    dma_channel_cleanup(channel_rx_DMA);
    dma_channel_unclaim(channel_rx_DMA);
    channel_rx_DMA = -1;
  }
  if (channel_tx_DMA > 0)
  {
    dma_channel_cleanup(channel_tx_DMA);
    dma_channel_unclaim(channel_tx_DMA);
  }

  spi_get_hw(spi)->dmacr = 0;
}

} //end namespace