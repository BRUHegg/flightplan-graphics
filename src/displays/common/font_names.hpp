#pragma once

#include <util/geom.hpp>

#include "cairo_utils.hpp"

namespace fms_display_fonts {

static const char MAIN_FONT_NAME[] = "main_font";

static const char* FONT_NAMES[] = {
  MAIN_FONT_NAME
};

constexpr double kSmallTextSlope = 0.02195121951219512;
constexpr double kMediumTextSlope = 0.036585365853658534;
constexpr double kLargeTextSlope = 0.05121951219512195;

void draw_left_text(cairo_t* cr, cairo_font_face_t* font_face,
                        std::string txt, geom::vect2_t pos,
                        geom::vect3_t color, double font_sz, 
                        geom::vect2_t canvas_sz, 
                        double slope=kMediumTextSlope);

void draw_right_text(cairo_t* cr, cairo_font_face_t* font_face,
                        std::string txt, geom::vect2_t pos,
                        geom::vect3_t color, double font_sz, 
                        geom::vect2_t canvas_sz,
                        double slope=kMediumTextSlope);

void draw_centered_text(cairo_t* cr, cairo_font_face_t* font_face,
                        std::string txt, geom::vect2_t pos,
                        geom::vect3_t color, double font_sz, 
                        geom::vect2_t canvas_sz,
                        double slope=kMediumTextSlope);
} // namespace fms_display_fonts
