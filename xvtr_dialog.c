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
#include <semaphore.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr

#include "band.h"
#include "bandstack.h"
#include "bpsk.h"
#include "mode.h"
#include "filter.h"
#include "receiver.h"
#include "transmitter.h"
#include "wideband.h"
#include "discovered.h"
#include "adc.h"
#include "dac.h"
#include "radio.h"
#include "vfo.h"
#include "main.h"
#include "protocol1.h"
#include "protocol2.h"
#include "tx_panadapter.h"
#include "xvtr_dialog.h"

static GtkWidget *title[BANDS+XVTRS];
static GtkWidget *min_frequency[BANDS+XVTRS];
static GtkWidget *max_frequency[BANDS+XVTRS];
static GtkWidget *lo_frequency[BANDS+XVTRS];
static GtkWidget *lo_error[BANDS+XVTRS];
static GtkWidget *disable_pa[BANDS+XVTRS];
static GtkWidget *pa_calibration[BANDS+XVTRS];

void save_xvtr () {
  int i;
  int b;
  const char *t;
  const char *minf;
  const char *maxf;
  const char *lof;
  const char *loerr;
  const char *txlof;
  const char *txloerr;
  for(i=BANDS;i<BANDS+XVTRS;i++) {
    BAND *xvtr=band_get_band(i);
    BANDSTACK *bandstack=xvtr->bandstack;
    t=gtk_entry_get_text(GTK_ENTRY(title[i]));
    strcpy(xvtr->title,t);
    if(strlen(t)!=0) {
      minf=gtk_entry_get_text(GTK_ENTRY(min_frequency[i]));
      xvtr->frequencyMin=(long long)(atof(minf)*1000000.0);
      maxf=gtk_entry_get_text(GTK_ENTRY(max_frequency[i]));
      xvtr->frequencyMax=(long long)(atof(maxf)*1000000.0);
      lof=gtk_entry_get_text(GTK_ENTRY(lo_frequency[i]));
      xvtr->frequencyLO=(long long)(atof(lof)*1000000.0);
      loerr=gtk_entry_get_text(GTK_ENTRY(lo_error[i]));
      xvtr->errorLO=atoll(loerr);
      xvtr->disablePA=gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(disable_pa[i]));
      for(b=0;b<bandstack->entries;b++) {
        BANDSTACK_ENTRY *entry=&bandstack->entry[b];
	switch(b) {
          case 0:
            entry->frequency=xvtr->frequencyMin;
	    break;
          case 1:
            entry->frequency=xvtr->frequencyMin + ((xvtr->frequencyMax-xvtr->frequencyMin)/2LL);
	    break;
          case 2:
            entry->frequency=xvtr->frequencyMax;
	    break;
        }
        entry->mode=USB;
        entry->filter=F6;
      }
//g_print("min=%s:%lld max=%s:%lld lo=%s:%lld\n",minf,xvtr->frequencyMin,maxf,xvtr->frequencyMax,lof,xvtr->frequencyLO);
    } else {
      xvtr->frequencyMin=0;
      xvtr->frequencyMax=0;
      xvtr->frequencyLO=0;
      xvtr->errorLO=0;
      xvtr->disablePA=0;
    }
  }
}

void update_receiver(int band) {
  int i;
  RECEIVER *rx;
  gboolean saved_ctun;
  for(i=0;i<MAX_RECEIVERS;i++) {
    rx=radio->receiver[i];
    if(rx!=NULL) {
      if(rx->band_a==band) {
        BAND *xvtr=band_get_band(band);
        rx->lo_a=xvtr->frequencyLO;
        rx->error_a=xvtr->errorLO;
        saved_ctun=rx->ctun;
        if(saved_ctun) {
          rx->ctun=FALSE;
        }
        frequency_changed(rx);
        if(saved_ctun) {
          rx->ctun=TRUE;
        }

        if(radio->transmitter!=NULL) {
          if(radio->transmitter->rx==rx) {
            update_tx_panadapter(radio);
          }
        }
      } else if(rx->band_b==band) {
        BAND *xvtr=band_get_band(band);
        rx->lo_b=xvtr->frequencyLO;
        rx->error_b=xvtr->errorLO;
        if(radio->transmitter!=NULL) {
          if(radio->transmitter->rx==rx) {
            update_tx_panadapter(radio);
          }
        }
      }
    }
  }
}

void min_frequency_cb(GtkEditable *editable,gpointer user_data) {
  int band=GPOINTER_TO_INT(user_data);
  BAND *xvtr=band_get_band(band);
  const char* minf=gtk_entry_get_text(GTK_ENTRY(min_frequency[band]));
  xvtr->frequencyMin=(long long)(atof(minf)*1000000.0);
  update_receiver(band);
}

void max_frequency_cb(GtkEditable *editable,gpointer user_data) {
  int band=GPOINTER_TO_INT(user_data);
  BAND *xvtr=band_get_band(band);
  const char* maxf=gtk_entry_get_text(GTK_ENTRY(max_frequency[band]));
  xvtr->frequencyMin=(long long)(atof(maxf)*1000000.0);
  update_receiver(band);
}

void lo_frequency_cb(GtkEditable *editable,gpointer user_data) {
  int band=GPOINTER_TO_INT(user_data);
  BAND *xvtr=band_get_band(band);
  const char* lof=gtk_entry_get_text(GTK_ENTRY(lo_frequency[band]));
  xvtr->frequencyLO=(long long)(atof(lof)*1000000.0);
  update_receiver(band);
}

void update_lo_frequency(int band, long long new_lo) {
  BAND *xvtr=band_get_band(band);

  if(radio->dialog!=NULL) {
    char temp[32];
    sprintf(temp,"%.6f", (double)(new_lo / 1000000.0));
    gtk_entry_set_text(GTK_ENTRY(lo_frequency[band]), temp);
  }
}

void lo_error_cb(GtkEditable *editable,gpointer user_data) {
  int band=GPOINTER_TO_INT(user_data);
  BAND *xvtr=band_get_band(band);
  const char* errorf=gtk_entry_get_text(GTK_ENTRY(lo_error[band]));
  xvtr->errorLO=atoll(errorf);
  update_receiver(band);
}

void lo_error_update(int band,long long offset) {
  BAND *xvtr=band_get_band(band);
  xvtr->errorLO=xvtr->errorLO+offset;
  update_receiver(band);
  if(radio->dialog!=NULL) {
    char temp[32];
    sprintf(temp,"%lld",xvtr->errorLO);
    gtk_entry_set_text(GTK_ENTRY(lo_error[band]),temp);
  }
}

static void pa_xvtr_value_changed_cb(GtkWidget *widget, gpointer user_data) {
  int band = GPOINTER_TO_INT(user_data);
  BAND *xvtr=band_get_band(band);
  xvtr->pa_calibration = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
  update_receiver(band);    
}
static void xvtr_pa_disable_changed_cb(GtkWidget *widget, gpointer user_data) {
  int band = GPOINTER_TO_INT(user_data);
  BAND *xvtr=band_get_band(band);
  xvtr->disablePA = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget));
  update_receiver(band);    
}

// HL2 clock 2 output can be used for transverters
// However, testing has shown that if the Versaclock IC sets the 
// frequency NOT in integer mode, spurs occur. This allows the option
// for linHPSDR to calculate the LO frequency based in an integer divisor
// for the versaclock PLL.
static void cl2_mode_cb(GtkWidget *widget, gpointer data) {
  HERMESLITE2 *hl2 = (HERMESLITE2 *)data;
  hl2->cl2_integer_mode = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget));
 
  // Update display and disable input of LO freq while in 
  // integer mode for CL2
  for(int i= BANDS; i < BANDS+XVTRS; i++) {
    gtk_widget_set_sensitive(lo_frequency[i], !hl2->cl2_integer_mode);
    BAND *xvtr=band_get_band(i);

    char temp[32];
    long long new_lo_freq = 0;
    if ((xvtr->frequencyLO > 0) && (hl2->cl2_integer_mode)) {
      // Calculate nearest LO value for integer divisor
      new_lo_freq = HL2cl2CalculateNearest(hl2, xvtr->frequencyLO);
      update_lo_frequency(i, new_lo_freq);
    }
  }
}

GtkWidget *create_xvtr_dialog(RADIO *radio) {
  int row;
  int col;
  int i;
  char temp[32];

  GtkWidget *grid=gtk_grid_new();
  gtk_grid_set_column_spacing (GTK_GRID(grid),10);

  row=0;
  col=0;

  if (radio->hl2 != NULL) {
    GtkWidget *cl2_label = gtk_label_new("Use CL2 in integer mode");
    gtk_grid_attach(GTK_GRID(grid), cl2_label, col++, row, 1, 1);
    GtkWidget *cl2 = gtk_check_button_new();
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(cl2), radio->hl2->cl2_integer_mode);
    gtk_grid_attach(GTK_GRID(grid), cl2, col++, row, 1, 1);
    g_signal_connect(cl2, "toggled", G_CALLBACK(cl2_mode_cb), radio->hl2);
  }

  row++;
  col=0;

  GtkWidget *label=gtk_label_new("Title");
  gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);
  label=gtk_label_new("Min Frequency(MHz)");
  gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);
  label=gtk_label_new("Max Frequency(MHz)");
  gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);
  label=gtk_label_new("LO Frequency(MHz)");
  gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);
  label=gtk_label_new("LO Error(Hz)");
  gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);
  label=gtk_label_new("Disable PA");
  gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);

  if (radio->hl2 != NULL) {
    label=gtk_label_new("PA Calibration");
    gtk_grid_attach(GTK_GRID(grid),label,col++,row,1,1);
  }

  row++;
  col=0;

  for(i=BANDS;i<BANDS+XVTRS;i++) {
    BAND *xvtr=band_get_band(i);

    title[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(title[i]),7);
    gtk_entry_set_text(GTK_ENTRY(title[i]),xvtr->title);
    gtk_grid_attach(GTK_GRID(grid),title[i],col++,row,1,1);

    min_frequency[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(min_frequency[i]),7);
    sprintf(temp,"%f",(double)xvtr->frequencyMin/1000000.0);
    gtk_entry_set_text(GTK_ENTRY(min_frequency[i]),temp);
    gtk_grid_attach(GTK_GRID(grid),min_frequency[i],col++,row,1,1);
    g_signal_connect(min_frequency[i],"changed",G_CALLBACK(min_frequency_cb),GINT_TO_POINTER(i));

    max_frequency[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(max_frequency[i]),7);
    sprintf(temp,"%f",(double)xvtr->frequencyMax/1000000.0);
    gtk_entry_set_text(GTK_ENTRY(max_frequency[i]),temp);
    gtk_grid_attach(GTK_GRID(grid),max_frequency[i],col++,row,1,1);
    g_signal_connect(max_frequency[i],"changed",G_CALLBACK(max_frequency_cb),GINT_TO_POINTER(i));

    lo_frequency[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(lo_frequency[i]),7);
    sprintf(temp,"%f",(double)xvtr->frequencyLO/1000000.0);
    gtk_entry_set_text(GTK_ENTRY(lo_frequency[i]),temp);
    gtk_grid_attach(GTK_GRID(grid),lo_frequency[i],col++,row,1,1);
    g_signal_connect(lo_frequency[i],"changed",G_CALLBACK(lo_frequency_cb),GINT_TO_POINTER(i));

    lo_error[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(lo_error[i]),9);
    sprintf(temp,"%lld",xvtr->errorLO);
    gtk_entry_set_text(GTK_ENTRY(lo_error[i]),temp);
    gtk_grid_attach(GTK_GRID(grid),lo_error[i],col++,row,1,1);
    g_signal_connect(lo_error[i],"changed",G_CALLBACK(lo_error_cb),GINT_TO_POINTER(i));

    disable_pa[i]=gtk_check_button_new();
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(disable_pa[i]),xvtr->disablePA);
    gtk_grid_attach(GTK_GRID(grid),disable_pa[i],col++,row,1,1);
    g_signal_connect(disable_pa[i], "toggled", G_CALLBACK(xvtr_pa_disable_changed_cb), GINT_TO_POINTER(i));

    if (radio->hl2 != NULL) {
      pa_calibration[i] = gtk_spin_button_new_with_range(38.8,100.0,0.1);
      gtk_spin_button_set_value(GTK_SPIN_BUTTON(pa_calibration[i]),(double)xvtr->pa_calibration);
      gtk_widget_show(pa_calibration[i]);    
      gtk_grid_attach(GTK_GRID(grid),pa_calibration[i],col++,row,1,1);
      g_signal_connect(pa_calibration[i],"changed",G_CALLBACK(pa_xvtr_value_changed_cb),GINT_TO_POINTER(i));

      if (radio->hl2->cl2_integer_mode) {
        g_print("Set disable pa false\n");
        gtk_widget_set_sensitive(disable_pa[i], FALSE);
      }
    }

    row++;
    col=0;

  }
  
  gtk_widget_show_all(grid);

  return grid;

}

