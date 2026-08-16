#include "font_names.hpp"

#include <algorithm>

#include <util/geom.hpp>

#include "cairo_utils.hpp"

namespace {

constexpr double kRefPx = 900.0;

double GetScaledFontSize(double font_sz, geom::vect2_t canvas_sz, 
  double slope) noexcept {
  double min_dim = std::min(canvas_sz.x, canvas_sz.y);
  double delta = (min_dim - kRefPx) * slope;
  return delta + font_sz;
}
} // namespace

namespace fms_display_fonts {

void draw_left_text(cairo_t* cr, cairo_font_face_t* font_face,
                        std::string txt, geom::vect2_t pos,
                        geom::vect3_t color, double font_sz, 
                        geom::vect2_t canvas_sz, double slope) {
  cairo_utils::draw_left_text(cr, font_face, txt, pos, 
    color, GetScaledFontSize(font_sz, canvas_sz, slope));    
}

void draw_right_text(cairo_t* cr, cairo_font_face_t* font_face,
                        std::string txt, geom::vect2_t pos,
                        geom::vect3_t color, double font_sz, 
                        geom::vect2_t canvas_sz, double slope) {
  cairo_utils::draw_right_text(cr, font_face, txt, pos, 
    color, GetScaledFontSize(font_sz, canvas_sz, slope));               
}

void draw_centered_text(cairo_t* cr, cairo_font_face_t* font_face,
                        std::string txt, geom::vect2_t pos,
                        geom::vect3_t color, double font_sz, 
                        geom::vect2_t canvas_sz, double slope) {
  cairo_utils::draw_centered_text(cr, font_face, txt, pos, 
    color, GetScaledFontSize(font_sz, canvas_sz, slope));
}
} // namespace fms_display_fonts
