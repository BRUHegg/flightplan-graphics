#pragma once

#include <chrono>
#include <memory>

#include <cairo/cairo.h>

#include <common/bytemap.hpp>
#include <common/cairo_utils.hpp>
#include <util/geom.hpp>

#include "cdu.hpp"

namespace fms_displays {

const std::string CDU_TEXTURE_NAME = "cdu";

class CDUWidget {
public:
  CDUWidget(geom::vect2_t pos, geom::vect2_t sz, 
             std::shared_ptr<cairo_utils::texture_manager_t> tm,
             std::shared_ptr<CDU> cdu, std::shared_ptr<CDUDisplay> cdu_displ, 
             byteutils::Bytemap* bm);

  void on_click(geom::vect2_t pos);

  void draw(cairo_t* cr);
private:
  std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr_;

  std::shared_ptr<CDUDisplay> cdu_displ_;
  std::shared_ptr<CDU> cdu_ptr_;
  byteutils::Bytemap* key_map_;

  geom::vect2_t screen_pos_;  // position of the CDU texture on the screen
  geom::vect2_t size_;
  geom::vect2_t texture_size_;
  geom::vect2_t texture_scale_;

  std::chrono::time_point<std::chrono::steady_clock> last_press_tp_;

  void draw_exec(cairo_t* cr);
};
} // namespace fms_displays
