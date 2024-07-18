#include <gtk/gtk.h>
#include "displays/common/cairo_utils.hpp"
#include "main_helpers.hpp"

#define UNUSED(x) (void)(x)

const std::string WINDOW_TITLE = "ND Display";
const std::string BOEING_FONT_NAME = "BoeingFont.ttf";


std::shared_ptr<test::CMDInterface> cmdint;

FT_Library lib;
FT_Face font;
cairo_font_face_t* boeing_font_face;


gboolean keypress_handler(GtkWidget *widget, GdkEventKey *event, gpointer data) {
    //UNUSED(widget);
    UNUSED(data);
    if (event->keyval == GDK_KEY_space){
        std::cout << "Right arrow\n";
        cmdint->avncs->fpl_sys->step_ctr(false, false);
        cmdint->update();

        gtk_widget_queue_draw(widget);
        return TRUE;
    }
    return FALSE;
}

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

    FT_Init_FreeType(&lib);

    bool font_loaded = false;
    if(libnav::does_file_exist(BOEING_FONT_NAME))
    {
        font_loaded = cairo_utils::load_font(BOEING_FONT_NAME, lib, &font, 
            &boeing_font_face);
    }
    else
    {
        std::cout << "Font file " << BOEING_FONT_NAME << " was not found.\n";
    }

    if(!font_loaded)
    {
        std::cout << "Failed to load font: " << BOEING_FONT_NAME << " . Aborting\n";
        exit(0);
    }

    GtkWidget *window;
    GtkWidget *darea;

    gtk_init(&argc, &argv);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    darea = gtk_drawing_area_new();
    gtk_container_add(GTK_CONTAINER(window), darea);

    gtk_widget_add_events(window, GDK_KEY_PRESS_MASK);


    g_signal_connect (G_OBJECT (window), "key_press_event",
        G_CALLBACK (keypress_handler), NULL);
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