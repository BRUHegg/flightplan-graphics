#include <cairo.h>
#include <gtk/gtk.h>
#include "displays/common/cairo_utils.hpp"
#include "main_helpers.hpp"

#define UNUSED(x) (void)(x)


static void do_drawing(cairo_t *);

static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr,
                              gpointer user_data)
{
    UNUSED(widget);
    UNUSED(user_data);
    do_drawing(cr);

    return FALSE;
}

static void do_drawing(cairo_t *cr)
{
    cairo_utils::draw_rect(cr, {10, 10}, {100, 50}, {0, 1, 0}, 5);
    cairo_utils::draw_line(cr, {10, 10}, {110, 60}, {1, 0, 0}, 5);
}

int main(int argc, char *argv[])
{
    test::CMDInterface cmdint;

    GtkWidget *window;
    GtkWidget *darea;

    gtk_init(&argc, &argv);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    darea = gtk_drawing_area_new();
    gtk_container_add(GTK_CONTAINER(window), darea);

    g_signal_connect(G_OBJECT(darea), "draw",
                     G_CALLBACK(on_draw_event), NULL);
    g_signal_connect(window, "destroy",
                     G_CALLBACK(gtk_main_quit), NULL);

    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(window), 400, 90);
    gtk_window_set_title(GTK_WINDOW(window), "GTK window");

    gtk_widget_show_all(window);

    gtk_main();

    return 0;
}