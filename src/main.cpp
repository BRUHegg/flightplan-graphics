#include <gtk/gtk.h>
#include "displays/common/cairo_utils.hpp"
#include "main_helpers.hpp"

#define UNUSED(x) (void)(x)

const std::string WINDOW_TITLE = "ND Display";

std::shared_ptr<test::CMDInterface> cmdint;


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
    cmdint->draw(cr);
}

int main(int argc, char *argv[])
{
    cmdint = std::make_shared<test::CMDInterface>();

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
    gtk_window_set_default_size(GTK_WINDOW(window), test::ND_SZ.x, test::ND_SZ.y);
    gtk_window_set_title(GTK_WINDOW(window), WINDOW_TITLE.c_str());

    gtk_widget_show_all(window);

    gtk_main();

    return 0;
}