/* Copyright (C)
* 2020 - m5evt
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

#include "puresignal.h"

#include "wdsp.h"

#include "agc.h"
#include "mode.h"
#include "filter.h"
#include "bandstack.h"
#include "band.h"
#include "discovered.h"
#include "bpsk.h"
#include "peak_detect.h"
#include "receiver.h"
#include "transmitter.h"
#include "wideband.h"
#include "adc.h"
#include "dac.h"
#include "radio.h"
#include "main.h"
#include "vfo.h"
#include "meter.h"
#include "radio_info.h"
#include "rx_panadapter.h"
#include "tx_panadapter.h"
#include "waterfall.h"
#include "protocol1.h"
#include "protocol2.h"
#include "audio.h"
#include "receiver_dialog.h"
#include "configure_dialog.h"
#include "property.h"
#include "rigctl.h"
#include "subrx.h"
#include "audio.h"
#include "math.h"

PSIGNAL *create_puresignal(void) {
  PSIGNAL *ps = g_new0(PSIGNAL, 1);

  ps->ints = 16;
  ps->spi = 256;
  ps->stbl = 0;
  ps->map = 1;
  ps->pin = 1;
  ps->ptol = 0.8;
  ps->mox_delay = 0.2;
  ps->loop_delay = 0.0;
  ps->amp_delay = 150E-9;

  ps->peak_value = 0;

  SetPSIntsAndSpi(radio->transmitter->channel, ps->ints, ps->spi);
  SetPSStabilize(radio->transmitter->channel, ps->stbl);
  SetPSMapMode(radio->transmitter->channel, ps->map);
  SetPSPinMode(radio->transmitter->channel, ps->pin);
  SetPSPtol(radio->transmitter->channel, ps->ptol);
  SetPSMoxDelay(radio->transmitter->channel, ps->mox_delay);
  SetPSTXDelay(radio->transmitter->channel, ps->amp_delay);
  SetPSLoopDelay(radio->transmitter->channel, ps->loop_delay);

  return ps;
}

