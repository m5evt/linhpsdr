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
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <wdsp.h>

#include "bpsk.h"
#include "receiver.h"
#include "transmitter.h"
#include "wideband.h"
#include "discovered.h"
#include "adc.h"
#include "dac.h"
#include "radio.h"
#include "main.h"
#include "protocol1.h"
#include "protocol2.h"
#include "audio.h"
#include "band.h"

static int deltadb=0;
static gboolean running=FALSE;

#define BLDR_RX 0
#define BLDR_CM 1
#define BLDR_CC 2
#define BLDR_CS 3
#define FEEDBACK 4
#define COR_CNT 5
#define SLN_CHK 6
#define DG_CNT 13
#define STATUS 15

#define INFO_SIZE 16
static int info[INFO_SIZE];
static int save_ps_auto;
static int save_ps_single;

static gboolean ps_configure_event_cb(GtkWidget *widget,GdkEventConfigure *event,gpointer data) {
  TRANSMITTER *tx=(TRANSMITTER *)data;
  gint width=gtk_widget_get_allocated_width(widget);
  gint height=gtk_widget_get_allocated_height(widget);
  if(tx->ps_surface) {
    cairo_surface_destroy(tx->ps_surface);
  }
  tx->ps_surface=gdk_window_create_similar_surface(gtk_widget_get_window(widget),CAIRO_CONTENT_COLOR,width,height);

  cairo_t *cr;
  cr = cairo_create (tx->ps_surface);
  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0);
  cairo_paint (cr);
  cairo_destroy(cr);
  return TRUE;
}

static gboolean ps_draw_cb(GtkWidget *widget,cairo_t *cr,gpointer data) {
  TRANSMITTER *tx=(TRANSMITTER *)data;
  if(tx->ps_surface!=NULL) {
    cairo_set_source_surface(cr,tx->ps_surface,0.0,0.0);
    cairo_paint(cr);
  }
  return FALSE;
}

static void update_ps(TRANSMITTER *tx,double pk) {
  cairo_t *cr;
  char text[32];

  if(tx->ps_surface!=NULL) {
    cr=cairo_create (tx->ps_surface);
    cairo_set_source_rgb(cr,1.0,1.0,1.0);
    cairo_paint(cr);

    cairo_set_font_size(cr,12);
    if(info[FEEDBACK]>181)  {
      cairo_set_source_rgb(cr,0.0,0.0,0.0);
    } else if(info[FEEDBACK]>128)  {
      cairo_set_source_rgb(cr,0.0,1.0,0.0);
    } else if(info[FEEDBACK]>90)  {
      cairo_set_source_rgb(cr,0.0,1.0,1.0);
    } else {
      cairo_set_source_rgb(cr,1.0,0.0,0.0);
    }
    cairo_move_to(cr,5,12);
    sprintf(text,"Feedback Level: %d",info[FEEDBACK]);
    cairo_show_text(cr,text);

    cairo_set_source_rgb(cr,0.0,0.0,0.0);

    cairo_move_to(cr,5,24);
    sprintf(text,"Correction Count: %d",info[COR_CNT]);
    cairo_show_text(cr,text);

    cairo_move_to(cr,5,36);
    sprintf(text,"Sln Chk: %d",info[SLN_CHK]);
    cairo_show_text(cr,text);

    cairo_move_to(cr,5,48);
    sprintf(text,"Dg Cnt: %d",info[DG_CNT]);
    cairo_show_text(cr,text);

    cairo_move_to(cr,5,60);
    switch(info[STATUS]) {
      case 0:
        cairo_show_text(cr,"STATUS: RESET");
        break;
      case 1:
        cairo_show_text(cr,"STATUS: WAIT");
        break;
      case 2:
        cairo_show_text(cr,"STATUS: MOXDELAY");
        break;
      case 3:
        cairo_show_text(cr,"STATUS: SETUP");
        break;
      case 4:
        cairo_show_text(cr,"STATUS: COLLECT");
        break;
      case 5:
        cairo_show_text(cr,"STATUS: MOXCHECK");
        break;
      case 6:
        cairo_show_text(cr,"STATUS: CALC");
        break;
      case 7:
        cairo_show_text(cr,"STATUS: DELAY");
        break;
      case 8:
        cairo_show_text(cr,"STATUS: STAY ON");
        break;
      case 9:
        cairo_show_text(cr,"STATUS: TURN ON");
        break;
      default:
        sprintf(text, "STATUS: UNKNOWN %d",info[STATUS]);
        cairo_show_text(cr,text);
        break;
    }

    cairo_move_to(cr,5,72);
    sprintf(text,"Peak: %f",pk);
    cairo_show_text(cr,text);

    cairo_destroy (cr);
    gtk_widget_queue_draw(tx->ps);
  }
}

static gboolean info_timeout(gpointer arg) {
  TRANSMITTER *tx=(TRANSMITTER *)arg;
  double pk;
  
  GetPSInfo(tx->channel,&info[0]);
  
  if (tx->puresignal == NULL) return FALSE;

  if (tx->puresignal->auto_on) {
    double fbk_db;
    int newcal = info[COR_CNT] != tx->puresignal->old_cor_cnt;
    tx->puresignal->old_cor_cnt = info[COR_CNT];

    int att = ps_get_tx_attenuation(tx->puresignal);
    
    switch(tx->puresignal->state) {
      case 0:
        if(newcal && ((info[FEEDBACK] > 175 && att < 31) || (info[FEEDBACK]<= 132 && att > 0))) {
          if(info[FEEDBACK] > 256) {
            fbk_db = 100;
          }
          else if (info[FEEDBACK] > 0) {
            fbk_db = 20.0 * log10((double)info[FEEDBACK] / 152.293);
          }
          else {
            fbk_db = -100.0;
          }
          fbk_db = (int)lround(fbk_db);
          ps_change_tx_attenuation(tx->puresignal, fbk_db);
        }
        break;
      case 1:
        tx->puresignal->state = 2;
        SetPSControl(tx->channel, 1, 0, 0, 0);
        break;
      case 2:
        tx->puresignal->state = 0;
        SetPSControl(tx->channel, 0, 0, 1, 0);
        break;
    }
  }
  GetPSMaxTX(tx->channel,&pk);
  update_ps(tx, pk);

  return running;
}

//static void enable_cb(GtkWidget *widget, gpointer data) {
//  TRANSMITTER *tx=(TRANSMITTER *)data;
//  transmitter_set_ps(tx,gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (widget)));
//}

static void twotone_cb(GtkWidget *widget, gpointer data) {
  TRANSMITTER *tx=(TRANSMITTER *)data;
  transmitter_set_twotone(tx,gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (widget)));
  if(tx->ps_twotone && (tx->puresignal != NULL)) {
    running=TRUE;
    tx->ps_timer_id=g_timeout_add(100,info_timeout,(gpointer)tx);
  } else {
    running=FALSE;
  }
}


GtkWidget *create_puresignal_dialog(TRANSMITTER *tx) {
  GtkWidget *grid=gtk_grid_new();
  gtk_grid_set_row_homogeneous(GTK_GRID(grid),FALSE);
  gtk_grid_set_column_homogeneous(GTK_GRID(grid),FALSE);
  gtk_grid_set_column_spacing(GTK_GRID(grid),5);
  gtk_grid_set_row_spacing(GTK_GRID(grid),5);

  int row=0;
  int col=0;

  GtkWidget *ps_frame=gtk_frame_new("Pure Signal");
  GtkWidget *ps_grid=gtk_grid_new();
  gtk_grid_set_column_spacing(GTK_GRID(ps_grid),10);
  gtk_grid_set_row_homogeneous(GTK_GRID(ps_grid),TRUE);
  gtk_grid_set_column_homogeneous(GTK_GRID(ps_grid),TRUE);
  gtk_container_add(GTK_CONTAINER(ps_frame),ps_grid);
  gtk_grid_attach(GTK_GRID(grid),ps_frame,col,row++,2,1);

  //GtkWidget *enable_b=gtk_check_button_new_with_label("Enable PS");
  //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (enable_b), tx->puresignal_enabled);
  //g_signal_connect(enable_b,"toggled",G_CALLBACK(enable_cb),tx);
  //gtk_grid_attach(GTK_GRID(ps_grid),enable_b,0,0,1,1);

  GtkWidget *twotone_b=gtk_check_button_new_with_label("Two Tone");
  g_signal_connect(twotone_b,"toggled",G_CALLBACK(twotone_cb),tx);
  gtk_grid_attach(GTK_GRID(ps_grid),twotone_b,1,0,1,1);

  tx->ps=gtk_drawing_area_new();
  g_signal_connect (tx->ps,"configure-event",G_CALLBACK(ps_configure_event_cb),(gpointer)tx);
  g_signal_connect (tx->ps,"draw",G_CALLBACK(ps_draw_cb),(gpointer)tx);
  gtk_grid_attach(GTK_GRID(ps_grid),tx->ps,0,1,8,8);

  return grid;
}
