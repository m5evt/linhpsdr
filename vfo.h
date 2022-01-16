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

extern GtkWidget *create_vfo(RECEIVER *rx);
extern void update_vfo(RECEIVER *r);



typedef struct _vfo_data {
  GtkWidget *vfo;
  GtkWidget *vfo_a_text;
  GtkWidget *a2b;
  GtkWidget *b2a;
  GtkWidget *aswapb;
  GtkWidget *split_b;
  GtkWidget *vfo_b_text;
  GtkWidget *zoom_b;
  GtkWidget *step_b;
  GtkWidget *tx_label;
  GtkWidget *frequency_a_text;
  GtkWidget *frequency_b_text;
  GtkWidget *subrx_b;
  GtkWidget *afgain_scale;
  GtkWidget *squelch_scale;
  GtkWidget *agcgain_scale;
  GtkWidget *squelch_label;
  GtkWidget *ps_b;
  GtkWidget *lock_b;
  GtkWidget *mode_b;
  GtkWidget *filter_b;
  GtkWidget *nb_b;
  GtkWidget *nr_b;
  GtkWidget *snb_b;
  GtkWidget *anf_b;
  GtkWidget *agc_b;
  GtkWidget *bmk_b;
  GtkWidget *ctun_b;
  GtkWidget *bpsk_b;
  GtkWidget *ant_b;
  GtkWidget *rit_b;
  GtkWidget *rit_value;
  GtkWidget *xit_b;
  GtkWidget *xit_value;
  GtkWidget *dup_b;
  GtkWidget *div_b;  
} VFO_DATA;

#define STEPS 16
extern gint64 steps[STEPS];
extern char *step_labels[STEPS];

extern void vfo_a2b(RECEIVER *rx);
extern void vfo_b2a(RECEIVER *rx);
extern void vfo_aswapb(RECEIVER *rx);

