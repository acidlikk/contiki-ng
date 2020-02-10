/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/stimer.h"
#include "sys/process.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "lpm.h"
#include "ieee-addr.h"
#include "ti-lib.h"

#include "apros-packet-control.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define ADC_SAMPLE 1024

static struct etimer et1, et2, et3;

uint8_t node_mac[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t frm_payload[150];

static uint16_t counter = 0;
uint16_t adc_temp = 0;
uint16_t adc_array[ADC_SAMPLE] = {0};

int current;
/*---------------------------------------------------------------------------*/
PROCESS(ct_waveform_virnect_process, "apros ct waveform for virnect process");
AUTOSTART_PROCESSES(&ct_waveform_virnect_process);
/*---------------------------------------------------------------------------*/
void input_callback(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest)
{
}
/*---------------------------------------------------------------------------*/
void
send_packet(uint8_t m_type, const uint8_t* payload, uint16_t payload_length)
{
  uint8_t making_packet[200];
  uint8_t packet_length;

  packet_length = make_packet(m_type, payload, payload_length, making_packet);

  nullnet_buf = making_packet;   // Point NullNet buffer to 'payload'
  nullnet_len = packet_length;   // Tell NullNet that the payload length is two bytes
  NETSTACK_NETWORK.output(NULL); // Send as broadcast
}
/*---------------------------------------------------------------------------*/
void
device_reset(void *ptr)
{
  watchdog_reboot();
}
void
gpio_init()
{
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_TPS_SW);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_TS_SW);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_TPL_DONE);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_CHA_ENABLE);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_TPL_DRV);

  ti_lib_gpio_set_dio(BOARD_IOID_CHA_ENABLE); // Harvesting OFF (Low Active)
  ti_lib_gpio_set_dio(BOARD_IOID_TS_SW); // Measuring ON
}
/*---------------------------------------------------------------------------*/
void
read_sensor(void)
{
  // Enable AUX
  // Read
  ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
  while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON))
  { }

  // Enable clocks
  ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
  while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY)
  { }

  // Select input
  AUXADCSelectInput(ADC_COMPB_IN_AUXIO1); // AD Channel
  
  // Configure and enable  
  AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_170_US, AUXADC_TRIGGER_MANUAL);
  //AUXADCEnableSync(AUXADC_REF_VDDS_REL, AUXADC_SAMPLE_TIME_85P3_US, AUXADC_TRIGGER_MANUAL);
    
  for(int i = 0; i < ADC_SAMPLE; i++) {
    AUXADCGenManualTrigger();
    adc_temp = AUXADCReadFifo();
    adc_array[i] = adc_temp;
  }
  
  // Disable ADC
  AUXADCDisable();
}
/*---------------------------------------------------------------------------*/
void
data_send(uint16_t index)
{
  // ID
  frm_payload[0] = FRM_TYPE_ID; frm_payload[1] = LENGTH_DATA_ID;
  frm_payload[2] = node_mac[0]; frm_payload[3] = node_mac[1]; frm_payload[4] = node_mac[2]; frm_payload[5] = node_mac[3];
  frm_payload[6] = node_mac[4]; frm_payload[7] = node_mac[5]; frm_payload[8] = node_mac[6]; frm_payload[9] = node_mac[7];
  // Index
  frm_payload[10] = FRM_TYPE_INDEX; frm_payload[11] = LENGTH_DATA_INDEX;
  frm_payload[12] = index & 0xFF; frm_payload[13] = (index >> 8) & 0xFF;
  // Data
  frm_payload[14] = FRM_TYPE_WAVEFORM; frm_payload[15] = 64;
  memcpy(&frm_payload[16], &adc_array[index * 32], 64);

  send_packet(MSG_TYPE_RAWDATA, frm_payload, 80);
}
/*---------------------------------------------------------------------------*/
void
data_fetch_ready()
{  
  // ID
  frm_payload[0] = FRM_TYPE_ID; frm_payload[1] = LENGTH_DATA_ID;
  frm_payload[2] = node_mac[0]; frm_payload[3] = node_mac[1]; frm_payload[4] = node_mac[2]; frm_payload[5] = node_mac[3];
  frm_payload[6] = node_mac[4]; frm_payload[7] = node_mac[5]; frm_payload[8] = node_mac[6]; frm_payload[9] = node_mac[7];
  // Data Fetch Ready
  frm_payload[10] = FRM_TYPE_READY; frm_payload[11] = LENGTH_DATA_READY;
  frm_payload[12] = 0; // status
  frm_payload[13] = 10; // sampling
  frm_payload[14] = 2; // byte unit
  frm_payload[15] = 1; // shaft
  frm_payload[16] = 31; // max index
  frm_payload[17] = 0; // max index

  send_packet(MSG_TYPE_CONTROL, frm_payload, 18);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ct_waveform_virnect_process, ev, data)
{
  PROCESS_BEGIN();

  nullnet_set_input_callback(input_callback);

  ieee_addr_cpy_to(node_mac, 8);

  gpio_init();
  ti_lib_gpio_set_dio(BOARD_IOID_TPS_SW); // Load Switch ON
  // always on
  
  // initialize
  etimer_set(&et1, CLOCK_SECOND >> 1);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
  read_sensor(); // garbage
  data_fetch_ready();

  etimer_set(&et1, CLOCK_SECOND >> 1);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));

  etimer_set(&et2, CLOCK_SECOND >> 1);
    
  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et2) {
        counter = 0;

        leds_on(LEDS_RED);
        read_sensor();
        leds_off(LEDS_RED);

        etimer_set(&et1, CLOCK_SECOND >> 1);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
        //
        etimer_set(&et3, CLOCK_SECOND >> 3); // 2
        etimer_set(&et2, CLOCK_SECOND * 30);
      } else if(data == &et3) {
        if(counter < 32) {
          data_send(counter);
          counter++;
          etimer_set(&et3, CLOCK_SECOND >> 3); // 2
        } else {
          ti_lib_gpio_set_dio(BOARD_IOID_TPL_DONE); // Timer Done
        }
      }
    }
    watchdog_periodic();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
