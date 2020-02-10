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
/**
 * \file
 * Packet process header filer
 * \author
 * Joonbum Kim
 *
 */
#ifndef APROS_PACKET_CONTROL_H_
#define APROS_PACKET_CONTROL_H_

/*---------------------------------------------------------------------------*/
/* SKT AI LEMS Protocol */
#define STX                   0x40
#define ETX                   0x7D

#define MSG_TYPE_REPORT       0x00
#define MSG_TYPE_CONTROL      0x01
#define MSG_TYPE_RAWDATA      0x02

#define FRM_TYPE_ID           0x01
#define FRM_TYPE_INDEX        0xAF
#define FRM_TYPE_WAVEFORM     0x35
#define FRM_TYPE_READY        0xA4

#define LENGTH_DATA_ID        0x08
#define LENGTH_DATA_INDEX     0x02
#define LENGTH_DATA_READY     0x06

#define RESPONSE_SUCCESS      0x00
#define RESPONSE_FAIL         0x01

/*---------------------------------------------------------------------------*/
extern uint16_t make_packet(uint8_t m_type, const uint8_t* payload, uint8_t payload_length, uint8_t* maked_packet);
extern uint8_t lems_packet_parser(uint8_t* packet_buffer, uint16_t packet_buffer_size, uint8_t* parsing_packet);

#endif /* APROS_PACKET_CONTROL_H_ */
