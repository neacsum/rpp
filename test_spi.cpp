#include <stdio.h>
#include "pico/stdlib.h"
#include <memory.h>

#include <rpp/spi.h>

// SPI Defines
// We are going to use SPI 1
#define SPI_PORT 1

// Allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define PIN_MISO 8
#define PIN_CS   9
#define PIN_SCK  14
#define PIN_MOSI 15



int main()
{
  const char *tx_str = "Hello world!"; //transmit string
  char rx_str[80]; //received string

  int i;

  stdio_init_all();

  //The SPI interface object
  RP2040::SPI spi (SPI_PORT, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);

  // SPI initialisation. This example will use SPI at 1MHz.
  spi.init (1E6);

  //set loopback mode for testing
  spi.loopback (true);

  //byte sized transfer
  for (i=0; tx_str[i]; ++i)
    rx_str[i] = spi.transfer (tx_str[i]);
  rx_str[i] = 0;
  printf ("Byte transfer received: %s\n", rx_str);

  //16-bit sized transfer
  const wchar_t *wtx_str = L"Hello worldwide!"; //wide version
  wchar_t wrx_str[80];
  for (i=0; wtx_str[i]; ++i)
    wrx_str[i] = spi.transfer16 (wtx_str[i]);
  wrx_str[i] = 0;
  //convert to narrow char to print it out (%ls is not supported)
  for (i=0; wrx_str[i]; ++i)
    rx_str[i] = wrx_str[i];
  printf ("16-bit transfer received: %s\n", rx_str);

  //DMA transfer
  memset (rx_str, 0, sizeof(rx_str));
  spi.transferDMA ((const uint8_t*)tx_str, (uint8_t*)rx_str, strlen(tx_str));
  while (!spi.finishedDMA ())
    ;

  printf ("DMA transfer received: %s\n", rx_str);

  spi.deinit();

  printf ("All done!\n");
}
