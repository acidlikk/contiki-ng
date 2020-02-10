/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
#include "contiki.h"
#include "apros-packet-control.h"
#include "lib/crc16.h" // crc
#include <string.h>

/*---------------------------------------------------------------------------*/
uint16_t
make_packet(uint8_t m_type, const uint8_t* payload, uint8_t payload_length, uint8_t* maked_packet)
{
  uint8_t packet_length;
  uint16_t crc;

  // STX
  maked_packet[0] = STX;
  // Length
  packet_length = 4 + payload_length;
  maked_packet[1] = packet_length;
  // Message-Type
  maked_packet[2] = m_type;
    
  // FRMPayload
  if(payload_length > 0) {
    memcpy(maked_packet + 3, payload, payload_length);
  }
    
  // CRC
  crc = crc16_data(maked_packet + 1, 2 + payload_length, 0); // little-endian
  maked_packet[3 + payload_length] = crc & 0xFF;
  maked_packet[4 + payload_length] = (crc >> 8) & 0xFF;
  // ETX
  maked_packet[5 + payload_length] = ETX;
    
  return (packet_length + 2);
}

/*---------------------------------------------------------------------------*/
uint8_t
lems_packet_parser(uint8_t* p_buf, uint16_t p_buf_size, uint8_t* parsing_packet)
{
  uint16_t start, length, end, crc;
  end = 0;
    
  for(start = 0; start < p_buf_size; start++) {
    if(p_buf[start] == STX) {
      // Data Length, Check
      length = p_buf[start + 1];
      length += 2; // STX + ETX
            
      if (start + length > p_buf_size) {
        return 0; // receive more
      }

      if(p_buf[start + length - 1] == ETX) { // ETX
        crc = crc16_data(p_buf + start + 1, length - 4, 0); // CRC
        if(crc == (p_buf[start + length - 2] << 8) + (p_buf[start + length - 3])) {
          // Control?
          if(p_buf[start + 2] == MSG_TYPE_CONTROL) {
            memcpy(parsing_packet, p_buf + start + 3, length - 6);
          }

          end = start + length;
                    
          if(start + length == p_buf_size) {
            return 1; // OK
          }
          else {
            start = end - 1;
            continue; // search more packet
          }
        }
        else { // CRC ERROR
          continue; // search more
        }
      }
      else { // WRONG ETX
        continue; // search more
      }
    }
  }
    
  return -1; // NO STX
}
