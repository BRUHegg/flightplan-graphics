/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This header file contains definitions of utility functions for cairo library. 
    Author: discord/bruh4096#4512(Tim G.)
*/


#pragma once

#include <cairo.h>
#include <cairo-ft.h>
#include <ft2build.h>
#include FT_FREETYPE_H


#include <string>
#include <map>
#include <vector>
#include "geom.hpp"


namespace cairo_utils
{
    // Colors:

    constexpr geom::vect3_t RED = {1, 0, 0};
    constexpr geom::vect3_t GREEN = {0, 1, 0};
    constexpr geom::vect3_t BLUE = {0, 0, 1};
    constexpr geom::vect3_t WHITE = {1, 1, 1};
    constexpr geom::vect3_t DARK_BLUE = {0.01, 0.05, 0.15};
    constexpr geom::vect3_t MAGENTA = {1, 0.451, 1};

    const std::string DEFAULT_IMG_FORMAT = ".png";


    struct texture_manager_t
    {
        std::map<std::string, cairo_surface_t*> data;


        bool load(std::vector<std::string> tx_names, std::string path, 
            std::string format = DEFAULT_IMG_FORMAT)
        {
            for(auto i: tx_names)
            {
                std::string full_path = path+i+format;

                cairo_surface_t *surf = cairo_image_surface_create_from_png(
                    full_path.c_str());
                
                if(surf == nullptr)
                    return false;

                data[i] = surf;
            }

            return true;
        }

        void destroy()
        {
            for(auto i: data)
            {
                cairo_surface_destroy(i.second);
            }
        }
    };


    inline bool load_font(std::string font_path, FT_Library ft_lib, FT_Face *font_face,
        cairo_font_face_t **cr_font)
    {
        FT_Error err = FT_New_Face(ft_lib, font_path.c_str(), 0, font_face);

        if (err)
            return false;
        
        *cr_font = cairo_ft_font_face_create_for_ft_face(*font_face, 0);

        return true;
    }

    inline void prepare_cairo_context(cairo_t* cr, geom::vect3_t color, int line_width)
    {
        cairo_set_source_rgb(cr, color.x, color.y, color.z);
        if(line_width > 0)
        {
            cairo_set_line_width(cr, line_width);
        }
        else
        {
            cairo_set_line_width(cr, 1);
        }
    }

    inline void draw_line(cairo_t* cr, geom::vect2_t start, geom::vect2_t end,
        geom::vect3_t color, double line_width=1)
    {
        prepare_cairo_context(cr, color, line_width);

        cairo_move_to(cr, start.x, start.y);
        cairo_line_to(cr, end.x, end.y);

        cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
        cairo_stroke(cr);
    }

    inline void draw_rect(cairo_t* cr, geom::vect2_t pos, geom::vect2_t sz, 
        geom::vect3_t color, double line_width=-1)
    {
        if(line_width > 0)
            prepare_cairo_context(cr, color, line_width);

        cairo_rectangle(cr, pos.x, pos.y, sz.x, sz.y);

        if(line_width > 0)
            cairo_stroke(cr);

        if(line_width <= 0)
        {
            cairo_fill(cr);
        }
    }

    inline void draw_tri(cairo_t* cr, geom::vect2_t v1, geom::vect2_t v2, geom::vect2_t v3, 
        geom::vect3_t color, double line_width=-1)
    {
        prepare_cairo_context(cr, color, line_width);

        cairo_move_to(cr, v1.x, v1.y);
        cairo_line_to(cr, v2.x, v2.y);
        cairo_line_to(cr, v3.x, v3.y);
        cairo_close_path(cr);

        cairo_stroke(cr);

        if(line_width < 0)
        {
            cairo_fill(cr);
        }
    }

    inline void draw_arc(cairo_t *cr, geom::vect2_t pos, double radius, double angle1, 
        double angle2, double line_width, geom::vect3_t color)
    {
        prepare_cairo_context(cr, color, line_width);

        cairo_move_to(cr, pos.x+radius, pos.y);
        cairo_arc(cr, pos.x, pos.y, radius, angle1, angle2);
        cairo_stroke(cr);

        if(line_width < 0)
        {
            cairo_fill(cr);
        }
    }

    inline void draw_circle(cairo_t *cr, geom::vect2_t pos, double radius, 
        double line_width, geom::vect3_t color)
    {
        draw_arc(cr, pos, radius, 0, 2 * M_PI, line_width, color);
    }

    inline void draw_left_text(cairo_t* cr, cairo_font_face_t *font_face, 
        std::string txt, geom::vect2_t pos, geom::vect3_t color, double font_sz)
    {
        const char *txt_cstr = txt.c_str();

        cairo_set_font_face(cr, font_face);
        cairo_set_source_rgb(cr, color.x, color.y, color.z);
        cairo_set_font_size(cr, font_sz);

        cairo_move_to(cr, pos.x, pos.y);
        cairo_show_text(cr, txt_cstr);
    }

    inline void draw_centered_text(cairo_t* cr, cairo_font_face_t *font_face, 
        std::string txt, geom::vect2_t pos, geom::vect3_t color, double font_sz)
    {
        const char *txt_cstr = txt.c_str();

        cairo_text_extents_t extents;
        cairo_set_font_face(cr, font_face);
        cairo_set_source_rgb(cr, color.x, color.y, color.z);
        cairo_set_font_size(cr, font_sz);
        cairo_text_extents(cr, txt_cstr, &extents);

        double x = pos.x-(extents.width/2 + extents.x_bearing);
        double y = pos.y-(extents.height/2 + extents.y_bearing);

        cairo_move_to(cr, x, y);
        cairo_show_text(cr, txt_cstr);
    }

    inline void draw_image(cairo_t* cr, cairo_surface_t *surf, geom::vect2_t pos, 
        geom::vect2_t scale, bool centered)
    {
        if(scale.x == 0 || scale.y == 0)
            return;

        cairo_save(cr);
        geom::vect2_t offset = {0, 0};

        if(centered)
        {
            offset.x = -cairo_image_surface_get_width(surf) * scale.x / 2;
            offset.y = -cairo_image_surface_get_height(surf) * scale.y / 2;
        }

        pos = pos + offset;

        cairo_set_source_surface(cr, surf, pos.x/scale.x, pos.y/scale.y);
        
        cairo_scale(cr, scale.x, scale.y);
                        
        cairo_paint(cr);
        cairo_restore(cr);
    }
}
