/*
        This project is licensed under
        Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
   Public License (CC BY-NC-SA 4.0).

        A SUMMARY OF THIS LICENSE CAN BE FOUND HERE:
   https://creativecommons.org/licenses/by-nc-sa/4.0/

        This is the entry point of the app.
    Author: discord/bruh4096#4512(Tim G.)
*/

#include <gtk/gtk.h>

#include "displays/common/cairo_utils.hpp"
#include "main_helpers.hpp"

#define UNUSED(x) (void)(x)

const std::string WINDOW_TITLE = "ND Display";

std::shared_ptr<fms_core::CMDInterface> cmdint;

gboolean keypress_handler(GtkWidget* widget, GdkEventKey* event,
                          gpointer data) {
  UNUSED(data);
  UNUSED(widget);

  if (event->keyval == GDK_KEY_Right) {
    cmdint->avncs->fpl_sys->step_ctr(false, 0);
  } else if (event->keyval == GDK_KEY_Left) {
    cmdint->avncs->fpl_sys->step_ctr(true, 0);
  } else if (event->keyval == GDK_KEY_Up) {
    cmdint->increment_nd_range(0);
  } else if (event->keyval == GDK_KEY_Down) {
    cmdint->decrement_nd_range(0);
  } else if (event->keyval == GDK_KEY_m || event->keyval == GDK_KEY_M) {
    cmdint->set_nd_mode(fms_core::NDMode::MAP, 0);
  } else if (event->keyval == GDK_KEY_p || event->keyval == GDK_KEY_P) {
    cmdint->set_nd_mode(fms_core::NDMode::PLAN, 0);
  } else if (event->keyval == GDK_KEY_a || event->keyval == GDK_KEY_A) {
    cmdint->toggle_bool_dr(fms_environment::ND_EFIS_AIRPORT_ON_VAR, 0);
  } else if (event->keyval == GDK_KEY_s || event->keyval == GDK_KEY_S) {
    cmdint->toggle_bool_dr(fms_environment::ND_EFIS_STATION_ON_VAR, 0);
  } else if (event->keyval == GDK_KEY_w || event->keyval == GDK_KEY_W) {
    cmdint->toggle_bool_dr(fms_environment::ND_EFIS_WAYPOINT_ON_VAR, 0);
  } else if (event->keyval == GDK_KEY_z || event->keyval == GDK_KEY_Z) {
    cmdint->switch_trk_hdg_up();
  } else if (event->keyval == GDK_KEY_r || event->keyval == GDK_KEY_R) {
    cmdint->avncs->fpl_sys->reset_ctr(0);
  }
  return FALSE;
}

static void do_drawing(cairo_t*);

static gboolean on_draw_event(GtkWidget* widget, cairo_t* cr,
                              gpointer user_data) {
  UNUSED(widget);
  UNUSED(user_data);
  do_drawing(cr);
  gtk_widget_queue_draw(widget);

  return FALSE;
}

static void do_drawing(cairo_t* cr) {
  cmdint->update();
  cmdint->draw(cr);
}

static gboolean clicked(GtkWidget* widget, GdkEventButton* event,
                        gpointer user_data) {
  UNUSED(user_data);

  cmdint->on_click({event->x, event->y});
  cmdint->update();
  gtk_widget_queue_draw(widget);

  return TRUE;
}

int main(int argc, char* argv[]) {
  try {
    cmdint = std::make_shared<fms_core::CMDInterface>();
  } catch(const std::exception& err) {
    std::cerr << err.what() << "\n";
    return 1;
  }
  

  GtkWidget* window;
  GtkWidget* darea;

  gtk_init(&argc, &argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  darea = gtk_drawing_area_new();
  gtk_container_add(GTK_CONTAINER(window), darea);

  gtk_widget_add_events(window, GDK_KEY_PRESS_MASK);

  g_signal_connect(G_OBJECT(window), "key_press_event",
                   G_CALLBACK(keypress_handler), NULL);
  g_signal_connect(window, "button-press-event", G_CALLBACK(clicked), NULL);
  g_signal_connect(G_OBJECT(darea), "draw", G_CALLBACK(on_draw_event), NULL);
  g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

  gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size(GTK_WINDOW(window), fms_core::WND_WIDTH,
                              fms_core::WND_HEIGHT);
  gtk_window_set_title(GTK_WINDOW(window), WINDOW_TITLE.c_str());

  gtk_widget_show_all(window);

  gtk_main();

  return 0;
}