/*
 * $Id$
 *
 * Paparazzi mcu1 spi functions
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/crc16.h>

#include "spi.h"

#define IT_PORT PORTD
#define IT_DDR  DDRD
#define IT_PIN  7

#define SPI_DDR  DDRB
#define SPI_MOSI_PIN 3
#define SPI_MISO_PIN 4
#define SPI_SCK_PIN  5

struct inter_mcu_msg from_mega128;
struct inter_mcu_msg to_mega128;
volatile bool_t mega128_receive_valid = FALSE;
volatile bool_t spi_was_interrupted = FALSE;

static volatile uint8_t idx_buf = 0;
static volatile uint16_t crc_in, crc_out;

void spi_init(void) {
  to_mega128.status = 0;
  to_mega128.nb_err = 0;

  /* set it pin output */
  //  IT_DDR |= _BV(IT_PIN);

  /* set MISO pin output */
  SPI_DDR |= _BV(SPI_MISO_PIN);
  /* enable SPI, slave, MSB first, sck idle low */
  SPCR = _BV(SPE);
  /* enable interrupt */
  SPCR |= _BV(SPIE);
}

void spi_reset(void) {
  idx_buf = 0;
  crc_in = CRC_INIT;
  crc_out = CRC_INIT;

  uint8_t first_byte = ((uint8_t*)&to_mega128)[0];
  crc_out = CrcUpdate(crc_out, first_byte);
  SPDR = first_byte;

  mega128_receive_valid = FALSE;
}

#include "uart.h"

/** c.f. autopilot/link_fbw.c */
SIGNAL(SIG_SPI) {
  static uint8_t tmp, crc_in1;
  
  idx_buf++;

  spi_was_interrupted = TRUE;

  if (idx_buf > FRAME_LENGTH)
    return;
  /* we have sent/received a complete frame */
  if (idx_buf == FRAME_LENGTH) {
    /* read second byte of crc from receive register */
    tmp = SPDR;
    /* notify valid frame  */
    if (crc_in1 == Crc1(crc_in) && tmp == Crc2(crc_in))
      mega128_receive_valid = TRUE;
    else
      to_mega128.nb_err++;
    return;
  }

  if (idx_buf == FRAME_LENGTH - 1) {
    /* send the second byte of the crc_out */
    tmp = Crc2(crc_out);
    SPDR = tmp;
    /* get the first byte of the crc_in */
    crc_in1 = SPDR;
    return;
  } 

  /* we are sending/receiving payload       */
  if (idx_buf < FRAME_LENGTH - 2) {
    /* place new payload byte in send register */
    tmp = ((uint8_t*)&to_mega128)[idx_buf];
    SPDR = tmp;
    crc_out = CrcUpdate(crc_out, tmp);
  } 
  /* we are done sending the payload */
  else { // idx_buf == FRAME_LENGTH - 2
    /* place first byte of crc_out */
    tmp = Crc1(crc_out);
    SPDR = tmp;
  }
  
  /* read the byte from receive register */
  tmp = SPDR;
  ((uint8_t*)&from_mega128)[idx_buf-1] = tmp;
  crc_in = CrcUpdate(crc_in, tmp);
}
