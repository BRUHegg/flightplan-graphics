#pragma once

#include <chrono>
#include <memory>

#include <cairo/cairo.h>

#include <displays/common/bytemap.hpp>
#include <displays/common/cairo_utils.hpp>
#include <displays/common/texture_manager.hpp>
#include <util/geom.hpp>
#include <util/util.hpp>

#include "cdu.hpp"

namespace fms_displays {

util::const_str_data_t GetCduWidgetTextureNames();

class CDUWidget final {
public:
  CDUWidget(geom::vect2_t pos, geom::vect2_t sz, 
             TextureManager* tm,
             CDU* cdu, CDUDisplay* cdu_displ, 
             byteutils::Bytemap* bm);

  void on_click(geom::vect2_t pos);

  void draw(cairo_t* cr);
private:
  cairo_surface_t* cdu_texture_;

  CDUDisplay* cdu_displ_;
  CDU* cdu_ptr_;
  byteutils::Bytemap* key_map_;

  geom::vect2_t screen_pos_;  // position of the CDU texture on the screen
  geom::vect2_t size_;
  geom::vect2_t texture_size_;
  geom::vect2_t texture_scale_;

  std::chrono::time_point<std::chrono::steady_clock> last_press_tp_;

  void draw_exec(cairo_t* cr);
};
} // namespace fms_displays
