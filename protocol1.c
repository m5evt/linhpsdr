/* Copyright (C)
* 2018 - John Melton, G0ORX/N6LYT
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
*/

#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <net/if_arp.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <wdsp.h>

#include "band.h"
#include "channel.h"
#include "discovered.h"
#include "bpsk.h"
#include "mode.h"
#include "filter.h"
#include "receiver.h"
#include "transmitter.h"
#include "wideband.h"
#include "adc.h"
#include "dac.h"
#include "radio.h"
#include "main.h"
#include "protocol1.h"
#include "audio.h"
#include "signal.h"
#include "vfo.h"
#include "transmitter.h"
//#include "vox.h"
#include "ext.h"
#include "error_handler.h"
#include "hl2.h"




#define min(x,y) (x<y?x:y)

#define SYNC0 0
#define SYNC1 1
#define SYNC2 2
#define C0 3
#define C1 4
#define C2 5
#define C3 6
#define C4 7

#define DATA_PORT 1024

#define SYNC 0x7F
#define OZY_BUFFER_SIZE 512
//#define OUTPUT_BUFFER_SIZE 1024

// ozy command and control
#define MOX_DISABLED    0x00
#define MOX_ENABLED     0x01

#define MIC_SOURCE_JANUS 0x00
#define MIC_SOURCE_PENELOPE 0x80
#define CONFIG_NONE     0x00
#define CONFIG_PENELOPE 0x20
#define CONFIG_MERCURY  0x40
#define CONFIG_BOTH     0x60
#define PENELOPE_122_88MHZ_SOURCE 0x00
#define MERCURY_122_88MHZ_SOURCE  0x10
#define ATLAS_10MHZ_SOURCE        0x00
#define PENELOPE_10MHZ_SOURCE     0x04
#define MERCURY_10MHZ_SOURCE      0x08
#define SPEED_48K                 0x00
#define SPEED_96K                 0x01
#define SPEED_192K                0x02
#define SPEED_384K                0x03
#define MODE_CLASS_E              0x01
#define MODE_OTHERS               0x00
#define ALEX_ATTENUATION_0DB      0x00
#define ALEX_ATTENUATION_10DB     0x01
#define ALEX_ATTENUATION_20DB     0x02
#define ALEX_ATTENUATION_30DB     0x03
#define LT2208_GAIN_OFF           0x00
#define LT2208_GAIN_ON            0x04
#define LT2208_DITHER_OFF         0x00
#define LT2208_DITHER_ON          0x08
#define LT2208_RANDOM_OFF         0x00
#define LT2208_RANDOM_ON          0x10
#define CLOCK_PRECISION 1E9

double last_time = 0;

static int data_socket;
static struct sockaddr_in data_addr;
static int data_addr_length;

static int output_buffer_size;

static unsigned char control_in[5]={0x00,0x00,0x00,0x00,0x00};

static gboolean running;
static long ep4_sequence;
static long ep6_sequence = 0;

static int current_rx=0;

static int mic_samples=0;
static int mic_sample_divisor=1;

static unsigned char output_buffer[OZY_BUFFER_SIZE];
static int output_buffer_index=8;
static int tx_output_buffer_index=8;

static int command=1;

enum {
  SYNC_0=0,
  SYNC_1,
  SYNC_2,
  CONTROL_0,
  CONTROL_1,
  CONTROL_2,
  CONTROL_3,
  CONTROL_4,
  LEFT_SAMPLE_HI,
  LEFT_SAMPLE_MID,
  LEFT_SAMPLE_LOW,
  RIGHT_SAMPLE_HI,
  RIGHT_SAMPLE_MID,
  RIGHT_SAMPLE_LOW,
  MIC_SAMPLE_HI,
  MIC_SAMPLE_LOW,
  SKIP
};
static int state=SYNC_0;

static GThread *receive_thread_id;
static void start_protocol1_thread();
static gpointer receive_thread(gpointer arg);
static void process_ozy_input_buffer(unsigned char  *buffer);
static void process_wideband_buffer(unsigned char  *buffer);
void ozy_send_buffer();

static void protocol1_tx_scheduler_monitor(void);

static unsigned char metis_buffer[1032];
static long send_sequence=-1;
static int metis_offset=8;

static int metis_write(unsigned char ep,unsigned char* buffer,int length);
static void metis_start_stop(int command);
static void metis_send_buffer(unsigned char* buffer,int length);
static void metis_restart();

#define COMMON_MERCURY_FREQUENCY 0x80
#define PENELOPE_MIC 0x80

#ifdef USBOZY
//
// additional defines if we include USB Ozy support
//
#include "ozyio.h"

static GThread *ozy_EP4_rx_thread_id;
static GThread *ozy_EP6_rx_thread_id;
static gpointer ozy_ep4_rx_thread(gpointer arg);
static gpointer ozy_ep6_rx_thread(gpointer arg);
static void start_usb_receive_threads();
static int ozyusb_write(char* buffer,int length);
#define EP6_IN_ID  0x86                         // end point = 6, direction toward PC
#define EP2_OUT_ID  0x02                        // end point = 2, direction from PC
#define EP6_BUFFER_SIZE 2048
static unsigned char usb_output_buffer[EP6_BUFFER_SIZE];
static unsigned char ep6_inbuffer[EP6_BUFFER_SIZE];
static unsigned char usb_buffer_block = 0;
#endif

void protocol1_stop() {
  metis_start_stop(0);
  running=FALSE;
}

void protocol1_run() {
  fprintf(stderr,"protocol1_run\n");

  start_protocol1_thread();
  
  for(int i=8;i<OZY_BUFFER_SIZE;i++) {
    output_buffer[i]=0;
  }

  metis_restart();
}

void protocol1_set_mic_sample_rate(int rate) {
  mic_sample_divisor=rate/48000;
}

void protocol1_init(RADIO *r) {
  fprintf(stderr,"protocol1_init\n");

  protocol1_set_mic_sample_rate(r->sample_rate);
  if(radio->local_microphone) {
    if(audio_open_input(r)!=0) {
      radio->local_microphone=FALSE;
    }
  }

#ifdef USBOZY
//
// if we have a USB interfaced Ozy device:
//
  if (radio->discovered->device == DEVICE_OZY) {
    fprintf(stderr,"protocol1_init: initialise ozy on USB\n");
    ozy_initialise();
    start_usb_receive_threads();
  }
  else
#endif

  //start_protocol1_thread();

}

#ifdef USBOZY
//
// starts the threads for USB receive
// EP4 is the wideband endpoint
// EP6 is the "normal" USB frame endpoint
//
static void start_usb_receive_threads()
{
  int rc;

  fprintf(stderr,"protocol1 starting USB receive thread: buffer_size=%d\n",radio->buffer_size);

  ozy_EP6_rx_thread_id = g_thread_new( "OZY EP6 RX", ozy_ep6_rx_thread, NULL);
  if( ! ozy_EP6_rx_thread_id )
  {
    fprintf(stderr,"g_thread_new failed for ozy_ep6_rx_thread\n");
    exit( -1 );
  }
}

//
// receive threat for USB EP4 (wideband) not currently used.
//
static gpointer ozy_ep4_rx_thread(gpointer arg)
{
}

//
// receive threat for USB EP6 (512 byte USB Ozy frames)
// this function loops reading 4 frames at a time through USB
// then processes them one at a time.
//
static gpointer ozy_ep6_rx_thread(gpointer arg) {
  int bytes;
  unsigned char buffer[2048];

  fprintf(stderr, "protocol1: USB EP6 receive_thread\n");
  running=TRUE;
 
  while (running)
  {
    bytes = ozy_read(EP6_IN_ID,ep6_inbuffer,EP6_BUFFER_SIZE); // read a 2K buffer at a time

    if (bytes == 0)
    {
      fprintf(stderr,"protocol1_ep6_read: ozy_read returned 0 bytes... retrying\n");
      continue;
    }
    else if (bytes != EP6_BUFFER_SIZE)
    {
      fprintf(stderr,"protocol1_ep6_read: OzyBulkRead failed %d bytes\n",bytes);
      perror("ozy_read(EP6 read failed");
      //exit(1);
    }
    else
// process the received data normally
    {
      process_ozy_input_buffer(&ep6_inbuffer[0]);
      process_ozy_input_buffer(&ep6_inbuffer[512]);
      process_ozy_input_buffer(&ep6_inbuffer[1024]);
      process_ozy_input_buffer(&ep6_inbuffer[1024+512]);
    }

  }
  // terminate
  //_exit(0);
}
#endif

static void start_protocol1_thread() {
  fprintf(stderr,"protocol1 starting receive thread: buffer_size=%d output_buffer_size=%d\n",radio->buffer_size,output_buffer_size);

  switch(radio->discovered->device) {
#ifdef USBOZY
    case DEVICE_OZY:
      break;
#endif
    default:
      data_socket=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
      if(data_socket<0) {
        perror("protocol1: create socket failed for data_socket\n");
        exit(-1);
      }

      int optval = 1;
      if(setsockopt(data_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval))<0) {
        perror("data_socket: SO_REUSEADDR");
      }
      if(setsockopt(data_socket, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval))<0) {
        perror("data_socket: SO_REUSEPORT");
      }
      optval = 6;  
      if(setsockopt(data_socket, SOL_SOCKET, SO_PRIORITY, &optval, sizeof(optval))<0) {
        perror("data_socket: SO_PRIORITY");
      }      

      // bind to the interface
      if(bind(data_socket,(struct sockaddr*)&radio->discovered->info.network.interface_address,radio->discovered->info.network.interface_length)<0) {
        perror("protocol1: bind socket failed for data_socket\n");
        exit(-1);
      }

      memcpy(&data_addr,&radio->discovered->info.network.address,radio->discovered->info.network.address_length);
      data_addr_length=radio->discovered->info.network.address_length;
      data_addr.sin_port=htons(DATA_PORT);
      break;
  }

  receive_thread_id = g_thread_new( "protocol1", receive_thread, NULL);
  if( ! receive_thread_id )
  {
    fprintf(stderr,"g_thread_new failed on receive_thread\n");
    exit( -1 );
  }
  fprintf(stderr, "receive_thread: id=%p\n",receive_thread_id);

}

static gpointer receive_thread(gpointer arg) {
  struct sockaddr_in addr;
  socklen_t length;
  unsigned char buffer[2048];
  int bytes_read;
  int ep;
  long sequence;

  fprintf(stderr, "protocol1: receive_thread\n");
  running=TRUE;

  length=sizeof(addr);
  while(running) {

    switch(radio->discovered->device) {
#ifdef USBOZY
      case DEVICE_OZY:
        // should not happen
        break;
#endif

      default:
        bytes_read=recvfrom(data_socket,buffer,sizeof(buffer),0,(struct sockaddr*)&addr,&length);
        if(bytes_read<0) {
          if(errno==EAGAIN) {
            error_handler("protocol1: receiver_thread: recvfrom socket failed","Radio not sending data");
          } else {
            error_handler("protocol1: receiver_thread: recvfrom socket failed",strerror(errno));
          }
          //running=FALSE;
          continue;
        }

        if(buffer[0]==0xEF && buffer[1]==0xFE) {
          switch(buffer[2]) {
            case 1:
              // get the end point
              ep=buffer[3]&0xFF;

              // get the sequence number
              sequence=((buffer[4]&0xFF)<<24)+((buffer[5]&0xFF)<<16)+((buffer[6]&0xFF)<<8)+(buffer[7]&0xFF);

              switch(ep) {
                case 6: // EP6
                  ep6_sequence++;
                  if(sequence!=ep6_sequence) {
                    g_print("EP6 ERROR packet %ld pc %ld\n", sequence, ep6_sequence);
                    ep6_sequence = sequence;                    
                    if (radio->hl2 != NULL) radio->hl2->ep6_error_ctr++;
                  }              
                  // process the data
                  process_ozy_input_buffer(&buffer[8]);
                  process_ozy_input_buffer(&buffer[520]);
                  protocol1_tx_scheduler_monitor();   
                  //if (!radio->hl2->overflow) {
                    full_tx_buffer(radio->transmitter, FALSE);
                  //}
                  //else {
                  //  g_print("O EP6 %ld\n", sequence);                    
                  //}
                  
                  break;
                case 4: // EP4
                  ep4_sequence++;
                  if(sequence!=ep4_sequence) {
                    ep4_sequence=sequence;
                  } else {
                    //int seq=(int)(sequence%32L);
                    if((sequence%32L)==0L) {
                      reset_wideband_buffer_index(radio->wideband);
                    }
                    process_wideband_buffer(&buffer[8]);
                    process_wideband_buffer(&buffer[520]);
                  }
                  break;
                default:
                  fprintf(stderr,"unexpected EP %d length=%d\n",ep,bytes_read);
                  break;
              }
              break;
            case 2:  // response to a discovery packet
              fprintf(stderr,"unexepected discovery response when not in discovery mode\n");
              break;
            default:
              fprintf(stderr,"unexpected packet type: 0x%02X\n",buffer[2]);
              break;
          }
        } else {
          fprintf(stderr,"received bad header bytes on data port %02X,%02X\n",buffer[0],buffer[1]);
        }
        break;
    }

  }

  fprintf(stderr,"EXIT: protocol1: receive_thread\n");
  return NULL;
}

static void process_control_bytes() {
  gboolean previous_ptt;
  // Unused - commented in case used in future
  //gboolean previous_dot;
  //gboolean previous_dash;

  gint tx_mode = transmitter_get_mode(radio->transmitter); 
  
  previous_ptt=radio->local_ptt;
  //previous_dot=radio->dot;
  //previous_dash=radio->dash;
  radio->ptt=(control_in[0]&0x01)==0x01;
  radio->dash=(control_in[0]&0x02)==0x02;
  radio->dot=(control_in[0]&0x04)==0x04;

  radio->local_ptt=radio->ptt;
  if ((tx_mode==CWL || tx_mode==CWU) && radio->cw_keyer_internal) {
    radio->local_ptt=radio->ptt|radio->dot|radio->dash;
  }

  if(previous_ptt!=radio->local_ptt) {
g_print("process_control_bytes: ppt=%d dot=%d dash=%d\n",radio->ptt,radio->dot,radio->dash);
    g_idle_add(ext_ptt_changed,(gpointer)radio);
  }

  if (radio->hl2 != NULL) {
    gboolean ack = (control_in[0]&0xFF) >> 7;      
    if (ack) {
      // ACK from HL2  
      HL2i2cProcessReturnValue(radio->hl2, control_in[0], control_in[1],
                               control_in[2], control_in[3],
                               control_in[4]);  
      return;
    }
  } 
    
  switch((control_in[0]>>3)&0x1F) {
    case 0:
      radio->adc_overload=(control_in[1]&0x01)==0x01;
      radio->IO1=(control_in[1]&0x02)==0x02;
      radio->IO2=(control_in[1]&0x04)==0x04;
      radio->IO3=(control_in[1]&0x08)==0x08;
      
      
      //HL2 Buffer over/underflow
      if (radio->hl2 != NULL) {
        //g_print("%d\n", isTransmitting(radio));
        
        //if (!isTransmitting(radio)) {
          radio->hl2->overflow = FALSE;        
          radio->hl2->underflow = FALSE;         
        //}
        if (isTransmitting(radio)) {
          int recovery = control_in[3];
          
          if (control_in[3] == 128) g_print("CLICK\n");
          
          recovery = ((control_in[3] & 0xC0) >> 6);
          //g_print("recovery %d\n", recovery);
          
          /*
          if ((control_in[3]&0x80) == 0x80) {
            g_print("Under\n");
            g_print("bval %d\n", (int)control_in[3]);            
          }
          if (((control_in[3]&0x80) == 0x80) && ((control_in[3]&0x40) == 0x40)) {
            g_print("Over\n");            
            g_print("bval %d\n", (int)control_in[3]);            
          }          
          */
          
          
          if (recovery == 3) radio->hl2->overflow = TRUE;
          if (recovery == 2) radio->hl2->underflow = TRUE;

          int msb = control_in[3]; 
          double fill_level = (double)msb * 16 * 1.0/48   ;     

          
          if (fill_level < 5) {         
            /*
            g_print("EP6 %ld\n", ep6_sequence);   
            g_print("Fill LOW %lf %d%d\n", fill_level, radio->hl2->underflow, radio->hl2->overflow );
            
            g_print("Resize buffer %d\n", radio->hl2->hl2_tx_buffer_size);
            */
            /*
            if (radio->hl2->hl2_tx_buffer_size < 42) {
              radio->hl2->hl2_tx_buffer_size++;
            }
            */
            //full_tx_buffer(radio->transmitter, TRUE);
            //full_tx_buffer(radio->transmitter, TRUE);
            full_tx_buffer(radio->transmitter, TRUE);                        
          }   
          
          /*
          if (fill_level > 42) {            
           g_print("Fill HIGH %lf %d%d\n", fill_level, radio->hl2->underflow, radio->hl2->overflow );
          }
          */           
          
          
          // Did the PTT change because of a buffer underflow?
          
          //if (overflow || underflow) g_print("TX IQ FIFO flag %d%d\n", underflow, overflow );
          
          /*
          if (radio->hl2->underflow) {
            g_print("--U %lf %ld\n", fill_level, ep6_sequence);
            g_print("bval %d\n", (int)control_in[3]);
                      g_print("recovery %d\n", recovery);
          }
          
          if (radio->hl2->overflow) {
            g_print("--O %lf\n", fill_level);   
                               g_print("bval %d\n", (int)control_in[3]);
                      g_print("recovery %d\n", recovery);
          }
          */ 
          //if (previous_ptt != radio->ptt) g_print("TX IQ FIFO flag %d%d\n", underflow, overflow );
          
        }    
      }
      
      if(radio->mercury_software_version!=control_in[2]) {
        radio->mercury_software_version=control_in[2];
        fprintf(stderr,"  Mercury Software version: %d (0x%0X)\n",radio->mercury_software_version,radio->mercury_software_version);
      }
      if(radio->penelope_software_version!=control_in[3]) {
        radio->penelope_software_version=control_in[3];

        
        if(radio->discovered->device!=DEVICE_HERMES_LITE2) {        
          fprintf(stderr,"  Penelope Software version: %d (0x%0X)\n",radio->penelope_software_version,radio->penelope_software_version);          
        }
      }
      if(radio->ozy_software_version!=control_in[4]) {
        radio->ozy_software_version=control_in[4];
        fprintf(stderr,"FPGA firmware version: %d.%d\n",radio->ozy_software_version/10,radio->ozy_software_version%10);
      }
      break;
    case 1:
      radio->transmitter->exciter_power=((control_in[1]&0xFF)<<8)|(control_in[2]&0xFF); // from Penelope or Hermes
      
      int adc = ((control_in[1]&0xFF)<<8)|(control_in[2]&0xFF);

      double this_temperature = (3.26 * ((double)adc/4096.0) - 0.5) / 0.01;    
      // Exponential moving average filter
      double alpha = 0.7;
      radio->transmitter->temperature = (alpha * this_temperature) + (1 - alpha) * radio->transmitter->temperature;
      
      radio->transmitter->alex_forward_power=((control_in[3]&0xFF)<<8)|(control_in[4]&0xFF); // from Alex or Apollo
      break;
    case 2:
      radio->transmitter->alex_reverse_power=((control_in[1]&0xFF)<<8)|(control_in[2]&0xFF); // from Alex or Apollo
      radio->AIN3=(control_in[3]<<8)+control_in[4]; // from Pennelope or Hermes
      break;
    case 3:
      radio->AIN4=(control_in[1]<<8)+control_in[2]; // from Pennelope or Hermes
      radio->AIN6=(control_in[3]<<8)+control_in[4]; // from Pennelope or Hermes
      break;
  }
}

static int nreceiver;
static int left_sample;
static int right_sample;
static short mic_sample;
static double left_sample_double;
static double right_sample_double;
static double fbk_left_sample_double;
static double fbk_right_sample_double;
static int nsamples;
static int iq_samples;

static void process_ozy_byte(int b) {
  int i,j;
  switch(state) {
    case SYNC_0:
      if(b==SYNC) {
        state++;
      }
      break;
    case SYNC_1:
      if(b==SYNC) {
        state++;
      }
      break;
    case SYNC_2:
      if(b==SYNC) {
        state++;
      }
      break;
    case CONTROL_0:
      control_in[0]=b;
      state++;
      break;
    case CONTROL_1:
      control_in[1]=b;
      state++;
      break;
    case CONTROL_2:
      control_in[2]=b;
      state++;
      break;
    case CONTROL_3:
      control_in[3]=b;
      state++;
      break;
    case CONTROL_4:
      control_in[4]=b;
      process_control_bytes();
      nreceiver=0;
      iq_samples=(512-8)/((radio->receivers*6)+2);
      nsamples=0;
      state++;
      break;
    case LEFT_SAMPLE_HI:
      left_sample=(int)((signed char)b<<16);
      state++;
      break;
    case LEFT_SAMPLE_MID:
      left_sample|=(int)((((unsigned char)b)<<8)&0xFF00);
      state++;
      break;
    case LEFT_SAMPLE_LOW:
      left_sample|=(int)((unsigned char)b&0xFF);
      left_sample_double=(double)left_sample/8388607.0; // 24 bit sample 2^23-1
      state++;
      break;
    case RIGHT_SAMPLE_HI:
      right_sample=(int)((signed char)b<<16);
      state++;
      break;
    case RIGHT_SAMPLE_MID:
      right_sample|=(int)((((unsigned char)b)<<8)&0xFF00);
      state++;
      break;
    case RIGHT_SAMPLE_LOW:
      right_sample|=(int)((unsigned char)b&0xFF);
      right_sample_double=(double)right_sample/8388607.0; // 24 bit sample 2^23-1
      //find receiver
      i=-1;
      for(j=0;j<radio->discovered->supported_receivers;j++) {
        if(radio->receiver[j]!=NULL) {
          i++;
          if(i==nreceiver) break;
        }
      }
#ifdef PURESIGNAL
      if (isTransmitting(radio) && (radio->transmitter->puresignal != NULL)
          && (( nreceiver == radio->discovered->ps_tx_fdbk_chan)
          || (nreceiver == radio->discovered->ps_tx_fdbk_chan - 1))) {
        if (nreceiver == (radio->discovered->ps_tx_fdbk_chan - 1)) {
          fbk_left_sample_double = left_sample_double;
          fbk_right_sample_double = right_sample_double;
        } else {
          // Must be pre-DAC TX feedback
          if (nreceiver != 3) g_print("RX %i: add ps samples\n", nreceiver);
          add_ps_iq_samples(radio->transmitter, left_sample_double, right_sample_double, fbk_left_sample_double, fbk_right_sample_double);
        }                                                            
      }
      else if(radio->receiver[j]!=NULL) {
#else
      if(radio->receiver[j]!=NULL) {
#endif
        g_mutex_lock(&radio->delete_rx_mutex); 
        add_iq_samples(radio->receiver[j], left_sample_double,right_sample_double);
        g_mutex_unlock(&radio->delete_rx_mutex); 
      }
      nreceiver++;
      if(nreceiver==radio->receivers) {
        state++;
      } else {
        state=LEFT_SAMPLE_HI;
      }
      break;
    case MIC_SAMPLE_HI:
      mic_sample=(short)(b<<8);
      state++;
      break;
    case MIC_SAMPLE_LOW:
      mic_sample|=(short)(b&0xFF);
      if(!radio->local_microphone) {
        mic_samples++;
        if(mic_samples>=mic_sample_divisor) { // reduce to 48000
          add_mic_sample(radio->transmitter,(float)mic_sample/32768.0);
          mic_samples=0;
        }
      }
      nsamples++;
      if(nsamples==iq_samples) {
        state=SYNC_0;
      } else {
        nreceiver=0;
        state=LEFT_SAMPLE_HI;
      }
      break;
  }
}

static void process_ozy_input_buffer(unsigned char  *buffer) {
  int i;
  if(radio->receivers>0) {
    for(i=0;i<512;i++) {
      process_ozy_byte(buffer[i]&0xFF);
    }
  }
}

// Send rx audio back to radio
void protocol1_audio_samples(RECEIVER *rx,short left_audio_sample,short right_audio_sample) {
  if(!isTransmitting(radio)) {
//    if(rx->mixed_audio==0) {
      rx->mixed_left_audio=left_audio_sample;
      rx->mixed_right_audio=right_audio_sample;
//    } else {
//      rx->mixed_left_audio+=left_audio_sample;
//      rx->mixed_right_audio+=right_audio_sample;
//    }
//    rx->mixed_audio++;
//    if(rx->mixed_audio>=radio->receivers) {
      output_buffer[output_buffer_index++]=rx->mixed_left_audio>>8;
      output_buffer[output_buffer_index++]=rx->mixed_left_audio;
      output_buffer[output_buffer_index++]=rx->mixed_right_audio>>8;
      output_buffer[output_buffer_index++]=rx->mixed_right_audio;
      output_buffer[output_buffer_index++]=0;
      output_buffer[output_buffer_index++]=0;
      output_buffer[output_buffer_index++]=0;
      output_buffer[output_buffer_index++]=0;
      if(output_buffer_index>=OZY_BUFFER_SIZE) {
        ozy_send_buffer();
        output_buffer_index=8;
      }
//      rx->mixed_audio=0;
//    }
  }
}

void protocol1_iq_samples(int isample,int qsample) { 
    output_buffer[tx_output_buffer_index++]=0;
    output_buffer[tx_output_buffer_index++]=0;
    output_buffer[tx_output_buffer_index++]=0;
    output_buffer[tx_output_buffer_index++]=0;    
    
    output_buffer[tx_output_buffer_index++]=isample>>8;
    output_buffer[tx_output_buffer_index++]=isample; 

    output_buffer[tx_output_buffer_index++]=qsample>>8;
    output_buffer[tx_output_buffer_index++]=qsample;

    if(tx_output_buffer_index>=OZY_BUFFER_SIZE) {
      tx_output_buffer_index=8;
      ozy_send_buffer();
    }
  //}
}

void protocol1_eer_iq_samples(int isample,int qsample,int lasample,int rasample) {
  if(isTransmitting(radio)) {
    output_buffer[output_buffer_index++]=lasample>>8;
    output_buffer[output_buffer_index++]=lasample;
    output_buffer[output_buffer_index++]=rasample>>8;
    output_buffer[output_buffer_index++]=rasample;

    output_buffer[output_buffer_index++]=isample>>8;
    output_buffer[output_buffer_index++]=isample;
    output_buffer[output_buffer_index++]=qsample>>8;
    output_buffer[output_buffer_index++]=qsample;
    if(output_buffer_index>=OZY_BUFFER_SIZE) {
      ozy_send_buffer();
      output_buffer_index=8;
    }
  }
}

// Microphone buffer dump called from audio.c
void protocol1_process_local_mic(RADIO *r) {
  int i;
  for(i=0;i<r->local_microphone_buffer_size;i++) {
    add_mic_sample(r->transmitter,r->local_microphone_buffer[i]);
  }
}

double read_time_now(void) {
  struct timespec thiscall;
  clock_gettime(CLOCK_MONOTONIC, &thiscall);
  // Calculate time it took
  // TODO sort out sec and nsec units to accurately compare time (at the moment wrap around on nsec causes problems)
  double this_time = (double)thiscall.tv_sec + (double)(thiscall.tv_nsec / CLOCK_PRECISION);
  return this_time;
}

static void protocol1_tx_scheduler_monitor(void) {
  // Calculate time gap between packets received from p1 radio
  // TODO sort out sec and nsec units to accurately compare time (at the moment wrap around on nsec causes problems)
  double this_time = read_time_now();
  radio->protocol1_timer = 1E3 * (this_time - last_time);
  last_time = this_time;
  
  if (radio->hl2 != NULL) {
    if (radio->protocol1_timer > 20) {
      g_print( "tdiff %lf\n", radio->protocol1_timer); 
      radio->hl2->late_packets++;
    }
  }   
}

static void process_wideband_buffer(unsigned char  *buffer) {
  int i;
  short sample;
  double sampledouble;
  for(i=0;i<512;i+=2) {
    sample = (short) ((buffer[i + 1] << 8) + (buffer[i] & 0xFF));
    sampledouble=(double)sample/32767.0;
    if(radio->wideband!=NULL) {
      add_wideband_sample(radio->wideband, sampledouble);
    }
  }
}

void ozy_send_buffer() {
  int i,j;
  int count;
  BAND *band;
  int nreceivers;
  RECEIVER *tx_receiver;

  output_buffer[SYNC0]=SYNC;
  output_buffer[SYNC1]=SYNC;
  output_buffer[SYNC2]=SYNC;
  // Multiple synchronised HL2s. Only send command to the primary HL2 (unless we 
  // specificy a specific HL2 later)
  //if ((radio->hl2 != NULL) && (radio->diversity_mixers > 0)) {
  //  output_buffer[SYNC2] = HL2_SYNC_MASK_PRIMARY;
  //}
  output_buffer[C0]=0x00;
  output_buffer[C1]=0x00;
  output_buffer[C2]=0x00;
  output_buffer[C3]=0x00;
  output_buffer[C4]=0x00;

  if(metis_offset==8) {
    output_buffer[C0]=0x00;
    output_buffer[C1]=0x00;
    switch(radio->sample_rate) {
      case 48000:
        output_buffer[C1]|=SPEED_48K;
        break;
      case 96000:
        output_buffer[C1]|=SPEED_96K;
        break;
      case 192000:
        output_buffer[C1]|=SPEED_192K;
        break;
      case 384000:
        output_buffer[C1]|=SPEED_384K;
        break;
    }

// set more bits for Atlas based device
// CONFIG_BOTH seems to be critical to getting ozy to respond
#ifdef USBOZY
    if ((radio->discovered->device == DEVICE_OZY) || (radio->discovered->device == DEVICE_METIS))
#else
    if (radio->discovered->device == DEVICE_METIS)
#endif
    {
      if (radio->atlas_mic_source) output_buffer[C1] |= PENELOPE_MIC;
      output_buffer[C1] |= CONFIG_BOTH;
      if (radio->atlas_clock_source_128mhz) output_buffer[C1] |= MERCURY_122_88MHZ_SOURCE;
      output_buffer[C1] |= ((radio->atlas_clock_source_10mhz & 3) << 2);
    }

    output_buffer[C2]=0x00;
    if(radio->classE) {
      output_buffer[C2]|=0x01;
    }
    if(radio->transmitter->rx!=NULL) {
      band=band_get_band(radio->transmitter->rx->band_a);
      if(isTransmitting(radio)) {
#ifdef USE_VFO_B_MODE_AND_FILTER
        if(radio->transmitter->rx->split) {
          band=band_get_band(radio->transmitter->rx->band_b);
        }
#endif
        output_buffer[C2]|=band->OCtx<<1;
        if(radio->tune) {
          if(radio->OCmemory_tune_time!=0) {
            struct timeval te;
            gettimeofday(&te,NULL);
            long long now=te.tv_sec*1000LL+te.tv_usec/1000;
            if(radio->tune_timeout>now) {
              output_buffer[C2]|=radio->oc_tune<<1;
            }
          } else {
 // SM4VEY
        //    output_buffer[C2]|=radio->oc_tune<<1;
          }
        }
      } else {
 //Rx
        output_buffer[C2]|=band->OCrx<<1;
      }
    }

// TODO - add Alex Attenuation and Alex Antenna
    output_buffer[C3]=0x00;
    // Hermes Lite 2 FPGA PSU clock toggle
    if(radio->hl2 != NULL) {
      if (radio->hl2->psu_clk == FALSE) { 
        output_buffer[C3]|=LT2208_RANDOM_ON;    
      }
    }
    else {
      if(radio->adc[0].random) {
        output_buffer[C3]|=LT2208_RANDOM_ON;
      }
    }
      if(radio->adc[0].dither) {
        output_buffer[C3]|=LT2208_DITHER_ON;
      }
    if(radio->adc[0].preamp) {
      output_buffer[C3]|=LT2208_GAIN_ON;
    }

    switch(radio->adc[0].antenna) {
      case 0:  // ANT 1
        break;
      case 1:  // ANT 2
        break;
      case 2:  // ANT 3
        break;
      case 3:  // EXT 1
        //output_buffer[C3]|=0xC0;
        output_buffer[C3]|=0xE0;
        break;
      case 4:  // EXT 2
        //output_buffer[C3]|=0xC0;
        output_buffer[C3]|=0xA0;
        break;
      case 5:  // XVTR
        output_buffer[C3]|=0xE0;
        break;
      default:
        break;
    }


// TODO - add Alex TX relay, duplex, receivers Mercury board frequency
    output_buffer[C4]=0x04;  // duplex
    
    if (radio->diversity_mixers > 0) output_buffer[C4] |= 0x80;
          
    nreceivers=radio->receivers-1;
    
    output_buffer[C4]|=nreceivers<<3;
    if(isTransmitting(radio)) {
      switch(radio->alex_tx_antenna) {
        case 0:  // ANT 1
          output_buffer[C4]|=0x00;
          break;
        case 1:  // ANT 2
          output_buffer[C4]|=0x01;
          break;
        case 2:  // ANT 3
          output_buffer[C4]|=0x02;
          break;
        default:
          break;
      }
    //} else {
    // SM4VEY Atlas Alex fix
    }
      switch(radio->adc[0].antenna) {
        case 0:  // ANT 1
          output_buffer[C4]|=0x00;
          break;
        case 1:  // ANT 2
          output_buffer[C4]|=0x01;
          break;
        case 2:  // ANT 3
          output_buffer[C4]|=0x02;
          break;
        case 3:  // EXT 1
        case 4:  // EXT 2
        case 5:  // XVTR
          switch(radio->alex_tx_antenna) {
            case 0:  // ANT 1
              output_buffer[C4]|=0x00;
              break;
            case 1:  // ANT 2
              output_buffer[C4]|=0x01;
              break;
            case 2:  // ANT 3
              output_buffer[C4]|=0x02;
              break;
          }
          break;
      }
    //}
//SM4VEY Atlas fix
  } else {
    switch(command) {
      case 1: // tx frequency
        output_buffer[C0]=0x02;
        long long f = transmitter_get_frequency(radio->transmitter);

        output_buffer[C1]=f>>24;
        output_buffer[C2]=f>>16;
        output_buffer[C3]=f>>8;
        output_buffer[C4]=f;
        break;
      case 2: // rx frequency
//        nreceivers=radio->receivers;
        if(current_rx<radio->discovered->supported_receivers) {
          output_buffer[C0]=0x04+(current_rx*2);
#ifdef PURESIGNAL
          if (isTransmitting(radio) && (radio->transmitter->puresignal != NULL)
             && ((current_rx == radio->discovered->ps_tx_fdbk_chan)
             || (current_rx == radio->discovered->ps_tx_fdbk_chan - 1))) {
              // Force 2 receivers used for PS during TX to be locked to
              // the TX frequency
              long long txFrequency = transmitter_get_frequency(radio->transmitter);
              output_buffer[C1]=txFrequency>>24;
              output_buffer[C2]=txFrequency>>16;
              output_buffer[C3]=txFrequency>>8;
              output_buffer[C4]=txFrequency;
            } else {
#endif
            //find receiver
            count=-1;
            for(j=0;j<radio->discovered->supported_receivers;j++) {
              if(radio->receiver[j]!=NULL) {
                count++;
                if(count==current_rx) break;
              }
            }
            RECEIVER *rx=radio->receiver[j];
            
            long long rx_frequency=0;
            if(rx!=NULL) {
              rx_frequency=rx->frequency_a-rx->lo_a+rx->error_a;
              if(rx->rit_enabled) {
                rx_frequency+=rx->rit;
              }
              if(rx->mode_a==CWU) {
                rx_frequency-=(long long)radio->cw_keyer_sidetone_frequency;
              } else if(rx->mode_a==CWL) {
                rx_frequency+=(long long)radio->cw_keyer_sidetone_frequency;
              }
            }
            
            output_buffer[C1]=rx_frequency>>24;
            output_buffer[C2]=rx_frequency>>16;
            output_buffer[C3]=rx_frequency>>8;
            output_buffer[C4]=rx_frequency;
#ifdef PURESIGNAL 
        }
#endif 
            current_rx++;
        }
        if(current_rx>=radio->discovered->supported_receivers) {
          current_rx=0;
        }
        break;
      case 3:
        {
        gint tx_mode = transmitter_get_mode(radio->transmitter);
        
        int level=0;
        // Always send TX drive level for CW mode
        if(isTransmitting(radio) || (tx_mode==CWL) || (tx_mode==CWU)) {
          BAND *band;
          if(radio->transmitter!=NULL) {
            if(radio->transmitter->rx!=NULL) {
#ifdef USE_VFO_B_MODE_AND_FILTER
              if(radio->transmitter->rx->split) {
                band=band_get_band(radio->transmitter->rx->band_b);
              } else {
#endif
                band=band_get_band(radio->transmitter->rx->band_a);
#ifdef USE_VFO_B_MODE_AND_FILTER
              }
#endif
            }
          }
    
          int power=0;
          if(isTransmitting(radio) || (tx_mode==CWL) || (tx_mode==CWU)) {
            if(radio->tune && !radio->transmitter->tune_use_drive) {
              power=(int)(radio->transmitter->drive/100.0*radio->transmitter->tune_percent);
            } else {
              power=(int)radio->transmitter->drive;
            }
          }

          double target_dbm = 10.0 * log10(power * 1000.0);
          double gbb=band->pa_calibration;
          target_dbm-=gbb;
          double target_volts = sqrt(pow(10, target_dbm * 0.1) * 0.05);
          double volts=min((target_volts / 0.8), 1.0);
          double actual_volts=volts*(1.0/0.98);
  
          if(actual_volts<0.0) {
            actual_volts=0.0;
          } else if(actual_volts>1.0) {
            actual_volts=1.0;
          }
  
          level=(int)(actual_volts*255.0);
        }

        output_buffer[C0]=0x12;
        output_buffer[C1]=level&0xFF;
        output_buffer[C2]=0x00;
        if(radio->mic_boost) {
          output_buffer[C2]|=0x01;
        }
        if(radio->mic_linein) {
          output_buffer[C2]|=0x02;
        }
        
        band=band_get_band(radio->transmitter->rx->band_a);
#ifdef USE_VFO_B_MODE_AND_FILTER
        if(isTransmitting(radio)) {
          if(radio->transmitter->rx->split) {
            band=band_get_band(radio->transmitter->rx->band_b);
          }
        }
#endif        
        
        if ((radio->hl2 != NULL) && (!band->disablePA)) {
          output_buffer[C2]|=0x2C;
        }
        else {
          if(radio->filter_board==APOLLO) {
            output_buffer[C2]|=0x2C;
          }
        }
        if(((radio->filter_board==APOLLO) || (radio->discovered->device==DEVICE_HERMES_LITE2)) && radio->tune) {
            output_buffer[C2]|=0x10;
        }
        
        output_buffer[C3]=0x00;
        if(radio->transmitter->rx->band_a==band6) {
          output_buffer[C3]=output_buffer[C3]|0x40; // Alex 6M low noise amplifier
        }

        // HL2 drive level is across whole of C3
        if (radio->hl2 == NULL) {
          if (band->disablePA) output_buffer[C3]=output_buffer[C3]|0x80; // disable PA
        }          
                  
        output_buffer[C4]=0x00;

        switch(radio->adc[0].filters) {
          case AUTOMATIC:
            // nothing to do as the firmware sets the filters
            break;
          case MANUAL:
            output_buffer[C2]|=0x40;
            switch(radio->adc[0].hpf) {
              case BYPASS:
              output_buffer[C2]|=0x40;  // MANUAL 
              output_buffer[C3]|=0x20;  // BYPASS all HPFs
              break;
              case HPF_1_5:
                output_buffer[C3]|=0x10;
                break;
              case HPF_6_5:
                output_buffer[C3]|=0x08;
                break;
              case HPF_9_5:
                output_buffer[C3]|=0x04;
                break;
              case HPF_13:
                output_buffer[C3]|=0x01;
                break;
              case  HPF_20:
                output_buffer[C3]|=0x02;
                break;
            }
            switch(radio->adc[0].lpf) {
              case LPF_160:
                output_buffer[C3]|=0x08;
                break;
              case LPF_80:
                output_buffer[C3]|=0x04;
                break;
              case LPF_60_40:
                output_buffer[C3]|=0x02;
                break;
              case LPF_30_20:
                output_buffer[C3]|=0x01;
                break;
              case  LPF_17_15:
                output_buffer[C3]|=0x40;
                break;
              case  LPF_12_10:
                output_buffer[C3]|=0x20;
                break;
              case  LPF_6:
                output_buffer[C3]|=0x10;
                break;
            }
            break;
          }
        }
        break;
      case 4:
        output_buffer[C0]=0x14;
        output_buffer[C1]=0x00;
        for(i=0;i<2;i++) {
          output_buffer[C1]|=(radio->adc[i].preamp<<i);
        }
        if(radio->mic_ptt_enabled==0) {
          output_buffer[C1]|=0x40;
        }
        if(radio->mic_bias_enabled) {
          output_buffer[C1]|=0x20;
        }
        if(radio->mic_ptt_tip_bias_ring) {
          output_buffer[C1]|=0x10;
        }
        output_buffer[C2]=0x00;
        output_buffer[C2]|=radio->linein_gain;
#ifdef PURESIGNAL
        if(isTransmitting(radio) && (radio->transmitter->puresignal != NULL)) {
          output_buffer[C2]|=0x40;
        }
#endif
        output_buffer[C3]=0x00;
  
        output_buffer[C4]=0x00;
        if(radio->discovered->device==DEVICE_HERMES_LITE2) {
          // HL2 full AD9866 gain range -12 dB (0) to 48 dB (60)
          output_buffer[C4]=0x40;
          // HL2 extends into [5:0] of this buffer          
          output_buffer[C4]|=(((int)radio->adc[0].attenuation + 12)&0x3F);
        } else if(radio->discovered->device==DEVICE_HERMES_LITE) {
          if(!radio->adc[0].enable_step_attenuation) {
            output_buffer[C4]=0x20;
          }        
        } else if(radio->discovered->device==DEVICE_HERMES || radio->discovered->device==DEVICE_ANGELIA || radio->discovered->device==DEVICE_ORION || radio->discovered->device==DEVICE_ORION2) {
          if(radio->adc[0].enable_step_attenuation) {
            output_buffer[C4]=0x20;
          }
          output_buffer[C4]|=(int)radio->adc[0].attenuation&0x1F;
        } else {
          output_buffer[C4]=0x00;
        }
        
        if ((radio->hl2 != NULL) && (radio->diversity_mixers > 0)) {
          output_buffer[SYNC2] = HL2_SYNC_MASK_PRIMARY;
        }
        
        break;
      case 5:
        output_buffer[C0]=0x16;
        output_buffer[C1]=0x00;
        if(radio->receivers>=2) {
          if(radio->discovered->device==DEVICE_HERMES || radio->discovered->device==DEVICE_ANGELIA || radio->discovered->device==DEVICE_ORION || radio->discovered->device==DEVICE_ORION2) {
            /*output_buffer[C1]=0x20|(int)radio->receiver[1]->attenuation;*/
          }
        }
        output_buffer[C2]=0x00;
        if(radio->cw_keys_reversed) {
          output_buffer[C2]|=0x40;
        }
        output_buffer[C3]=radio->cw_keyer_speed | (radio->cw_keyer_mode<<6);
        output_buffer[C4]=radio->cw_keyer_weight | (radio->cw_keyer_spacing<<7);
        break;
      case 6:
        // need to add tx attenuation and rx ADC selection
        output_buffer[C0]=0x1C;
        output_buffer[C1]=0x00;
        
        if(radio->receiver[0]!=NULL) {
          output_buffer[C1]|=radio->receiver[0]->adc;
        }
        if(radio->receiver[1]!=NULL) {
          output_buffer[C1]|=(radio->receiver[1]->adc<<2);
        }
        if(radio->receiver[2]!=NULL) {
          output_buffer[C1]|=(radio->receiver[2]->adc<<4);
        }
        if(radio->receiver[3]!=NULL) {
          output_buffer[C1]|=(radio->receiver[3]->adc<<6);
        }
        output_buffer[C2]=0x00;
        if(radio->receiver[4]!=NULL) {
          output_buffer[C2]|=(radio->receiver[4]->adc);
        }
        if(radio->receiver[5]!=NULL) {
          output_buffer[C2]|=(radio->receiver[5]->adc<<2);
        }
        if(radio->receiver[6]!=NULL) {
          output_buffer[C2]|=(radio->receiver[6]->adc<<4);
        }
        
        output_buffer[C3]=0x00;
        output_buffer[C3]|=radio->transmitter->attenuation;
        // Enabled HL2 hardware managed LNA gain during TX
        if (radio->hl2 != NULL) {
          // HL2 full AD9866 gain range -12 dB (0) to 48 dB (60)
          // bit 5 enabled to turn on 20 dB attenuator
          // leave bit 7 as 0 for software controlled gain during tx
          //output_buffer[C3]=0x60;
          output_buffer[C3]=0xC0;
          // HL2 extends into [5:0] of this buffer          
          output_buffer[C3] |= (((int)radio->hl2->lna_gain_tx + 12) & 0x3F);
        }
        output_buffer[C4]=0x00;
        break;
      case 7:
        output_buffer[C0]=0x1E;
        output_buffer[C1]=0x00;
        
        gint tx_mode = transmitter_get_mode(radio->transmitter);
        if(tx_mode!=CWU && tx_mode!=CWL) {
          output_buffer[C1]|=0x00;
        } else {
          if(radio->tune || radio->vox || !radio->cw_keyer_internal) {
            output_buffer[C1]|=0x00;
          } else {
            // Enabled internal keyer (radio generated cw) also enables
            // cwx (no longer implemented in linHPSDR)
            output_buffer[C1]|=0x01;
          }
        }
        
        output_buffer[C2]=radio->cw_keyer_sidetone_volume;
        output_buffer[C3]=radio->cw_keyer_ptt_delay;
        output_buffer[C4]=0x00;
        break;
      case 8:
        output_buffer[C0]=0x20;
        output_buffer[C1]=(radio->cw_keyer_hang_time>>2) & 0xFF;
        output_buffer[C2]=radio->cw_keyer_hang_time & 0x03;
        output_buffer[C3]=(radio->cw_keyer_sidetone_frequency>>4) & 0xFF;
        output_buffer[C4]=radio->cw_keyer_sidetone_frequency & 0x0F;
        break;
      case 9:
        output_buffer[C0]=0x22;
        output_buffer[C1]=(radio->transmitter->eer_pwm_min>>2) & 0xFF;
        output_buffer[C2]=radio->transmitter->eer_pwm_min & 0x03;
        output_buffer[C3]=(radio->transmitter->eer_pwm_max>>2) & 0xFF;
        output_buffer[C4]=radio->transmitter->eer_pwm_max & 0x03;
        break;
      case 10:
        output_buffer[C0]=0x24;
        output_buffer[C1]=0x00;
        if(isTransmitting(radio)) {
          output_buffer[C1]|=0x80; // ground RX1 on transmit
        }
        output_buffer[C2]=0x00;
        if(radio->alex_rx_antenna==5) { // XVTR
          output_buffer[C2]=0x02;
        }
        output_buffer[C3]=0x00;
        output_buffer[C4]=0x00;
        break;
      case 11:
        //g_mutex_lock(&hl2i2c_mutex);
        if (radio->hl2 != NULL) {
          //g_mutex_lock(&hl2->i2c_mutex);
          // Is there anything in the PC to HL2 command ring buffer?
          if (HL2i2cWriteQueued(radio->hl2)) { 
            //g_print("-----I2C send to HL2\n");
            output_buffer[C0] = HL2i2cSendRqst(radio->hl2);                         
            //g_print("%x", output_buffer[C0]);
            output_buffer[C1] = HL2i2cReadWrite(radio->hl2);      
            //g_print("%x", output_buffer[C1]);  
            output_buffer[C2] = HL2i2cSendTargetAddr(radio->hl2);    
            //g_print("%x", output_buffer[C2]);                
            output_buffer[C3] = HL2i2cSendCommand(radio->hl2);        
            //g_print("%x", output_buffer[C3]);            
            output_buffer[C4] = HL2i2cSendValue(radio->hl2);
            //g_print("%x\n", output_buffer[C4]);     
            //g_print("-----I2C send done\n");                 
          }
          else if (radio->hl2->adc2_value_to_send) {
            // Coherent rx/diversity, send to the secondary HL2
            g_print("Send LNA2 gain\n");
            // Note - this will cause complications is PureSignal is implemented later
            output_buffer[C0]=0x14;
            output_buffer[C1]=0x00;
            output_buffer[C2]=0x00;
            output_buffer[C3]=0x00;
  
            // HL2 full AD9866 gain range -12 dB (0) to 48 dB (60)
            output_buffer[C4]=0x40;
            // HL2 extends into [5:0] of this buffer          
            output_buffer[C4] |= (((int)radio->hl2->adc2_lna_gain + 12) & 0x3F);
       
            output_buffer[SYNC2] = HL2_SYNC_MASK_SECONDARY;            
            radio->hl2->adc2_value_to_send = FALSE;
          }
          else {
            // TX buffer size
            output_buffer[C0]=0x2E;
            //output_buffer[C0]=(0x17>>1) & 0xFF;
            output_buffer[C1]=0x0;
            output_buffer[C2]=0x0;
            // PTT delay
            output_buffer[C3]=0x6;
            // TX buffer latency
            output_buffer[C4] = hl2_get_txbuffersize(radio->hl2);            
          } 
        }
        //g_mutex_unlock(&hl2->i2c_mutex);
        break;        
    }

    if(current_rx==0) {
      command++;
      if (radio->discovered->device==DEVICE_HERMES_LITE2) {
        if (command>11) {
          command=1;
        }
      }
      else {
        if (command>10) command=1;
      }
    }
  }

  // set mox
  gint tx_mode = transmitter_get_mode(radio->transmitter);   
  
  if ((tx_mode==CWU || tx_mode==CWL) && radio->cw_keyer_internal) {
    if(radio->tune) {
      output_buffer[C0]|=0x01;
    }
  } else {
    if(isTransmitting(radio)) {
      output_buffer[C0]|=0x01;
    }
  }

#ifdef USBOZY
//
// if we have a USB interfaced Ozy device:
//
  if (radio->discovered->device == DEVICE_OZY)
        ozyusb_write(output_buffer,OZY_BUFFER_SIZE);
  else
#endif
  metis_write(0x02,output_buffer,OZY_BUFFER_SIZE);

  //fprintf(stderr,"ozy_send_buffer: C0=%02X C1=%02X C2=%02X C3=%02X C4=%02X\n",
  //                output_buffer[C0],output_buffer[C1],output_buffer[C2],output_buffer[C3],output_buffer[C4]);
}

#ifdef USBOZY
static int ozyusb_write(char* buffer,int length)
{
  int i;

// batch up 4 USB frames (2048 bytes) then do a USB write
  switch(usb_buffer_block++)
  {
    case 0:
    default:
      memcpy(usb_output_buffer, buffer, length);
      break;

    case 1:
      memcpy(usb_output_buffer + 512, buffer, length);
      break;

    case 2:
      memcpy(usb_output_buffer + 1024, buffer, length);
      break;

    case 3:
      memcpy(usb_output_buffer + 1024 + 512, buffer, length);
      usb_buffer_block = 0;           // reset counter
// and write the 4 usb frames to the usb in one 2k packet
      i = ozy_write(EP2_OUT_ID,usb_output_buffer,EP6_BUFFER_SIZE);
      if(i != EP6_BUFFER_SIZE)
      {
        perror("protocol1: OzyWrite ozy failed");
      }
      break;
  }
}
#endif

static int metis_write(unsigned char ep,unsigned char* buffer,int length) {
  int i;

  // copy the buffer over
  for(i=0;i<512;i++) {
    metis_buffer[i+metis_offset]=buffer[i];
  }

  if(metis_offset==8) {
    metis_offset=520;
  } else {
    send_sequence++;
    metis_buffer[0]=0xEF;
    metis_buffer[1]=0xFE;
    metis_buffer[2]=0x01;
    metis_buffer[3]=ep;
    metis_buffer[4]=(send_sequence>>24)&0xFF;
    metis_buffer[5]=(send_sequence>>16)&0xFF;
    metis_buffer[6]=(send_sequence>>8)&0xFF;
    metis_buffer[7]=(send_sequence)&0xFF;


    // send the buffer
    metis_send_buffer(&metis_buffer[0],1032);
    metis_offset=8;

  }

  return length;
}

static void metis_restart() {
fprintf(stderr,"metis_restart\n");
  // reset metis frame
  metis_offset=8;

  // reset current rx
  current_rx=0;

  // send commands twice
  command=1;
  do {
    ozy_send_buffer();
  } while (command!=1);

  do {
    ozy_send_buffer();
  } while (command!=1);

  usleep(20000);

  // start the data flowing
  if(radio->wideband!=NULL) {
    metis_start_stop(3); 
  }
  else {
    metis_start_stop(1); // IQ data (wideband data disabled, set to 1)    
  }
}

static void metis_start_stop(int command) {
  int i;
  unsigned char buffer[64];
    
  state=SYNC_0;

#ifdef USBOZY
  if(radio->discovered->device!=DEVICE_OZY) {
#endif

  buffer[0]=0xEF;
  buffer[1]=0xFE;
  buffer[2]=0x04;    // start/stop command
  buffer[3]=command;    // send EP6 and EP4 data (0x00=stop)

  for(i=0;i<60;i++) {
    buffer[i+4]=0x00;
  }

  metis_send_buffer(buffer,sizeof(buffer));
#ifdef USBOZY
  }
#endif
}

static void metis_send_buffer(unsigned char* buffer,int length) {
  if(sendto(data_socket,buffer,length,0,(struct sockaddr*)&data_addr,data_addr_length)!=length) {
    perror("sendto socket failed for metis_send_data\n");
  }
}

gboolean protocol1_is_running() {
  return running;
}
