#include "cdu_widget.hpp"

#include <chrono>
#include <memory>

#include <cairo/cairo.h>

#include <displays/common/bytemap.hpp>
#include <displays/common/cairo_utils.hpp>
#include <util/geom.hpp>
#include <util/util.hpp>

namespace {

constexpr double CDU_PRS_INTV_SEC = 0.001;  // Threshold to skip gtk's double-clicks

constexpr geom::vect2_t EXEC_LT_POS = {0.793, 0.582 * 0.964};
constexpr geom::vect2_t EXEC_LT_SZ = {0.08 * 1.2 * 0.75,
                                      0.7 * 0.08 * 1.08 * 0.37 * 0.5};
constexpr geom::vect3_t EXEC_LT_CLR = {0.955, 0.906, 0.269};

// Texture names:
const char CDU_WIDGET_TEXTURE_NAME[] = "cdu";

const char* CDU_WIDGET_TEXTURE_ARRAY[] = {
  CDU_WIDGET_TEXTURE_NAME
};
} // namespace

namespace fms_displays {

util::const_str_data_t GetCduWidgetTextureNames() {
  return util::const_str_data_t{
    .ptr=CDU_WIDGET_TEXTURE_ARRAY, .size=MY_ARRAY_SIZE(CDU_WIDGET_TEXTURE_ARRAY)};
}

CDUWidget::CDUWidget(geom::vect2_t pos, geom::vect2_t sz, 
             util::OpaquePointer<TextureManager> tm,
             util::OpaquePointer<CDU> cdu, 
             util::OpaquePointer<CDUDisplay> cdu_displ, 
             util::OpaquePointer<byteutils::Bytemap> bm) : cdu_displ_{cdu_displ},
             cdu_ptr_{cdu}, key_map_{bm}, screen_pos_{pos}, size_{sz} {
  cdu_texture_ = tm->GetTexture(CDU_WIDGET_TEXTURE_NAME);
  assert(cdu_texture_ != nullptr);
  
  texture_size_ = cairo_utils::get_surf_sz(cdu_texture_);
  texture_scale_ = size_ / texture_size_;

  last_press_tp_ = std::chrono::steady_clock::now();
}

void CDUWidget::on_click(geom::vect2_t pos) {
  auto curr_tp = std::chrono::steady_clock::now();
  std::chrono::duration<double> dur = curr_tp - last_press_tp_;
  double dur_cnt = dur.count();

  last_press_tp_ = curr_tp;

  if (dur_cnt < CDU_PRS_INTV_SEC) return;

  pos = (pos - screen_pos_) / texture_scale_;
  if (pos.x >= 0 && pos.y >= 0 && pos.x < texture_size_.x && pos.y < texture_size_.y) {
    int event = int(key_map_->get_at(size_t(pos.x), size_t(pos.y)));

    cdu_displ_->on_event(event);
  }
}

void CDUWidget::draw(cairo_t* cr) {
  cairo_utils::draw_image(cr, cdu_texture_, screen_pos_,
                          texture_scale_, false);

  bool dr_exc = cdu_ptr_->get_exec_lt();
  if (dr_exc) draw_exec(cr);

  cdu_displ_->draw(cr);
}

// Private member functions:
void CDUWidget::draw_exec(cairo_t* cr) {
  geom::vect2_t lt_pos = {screen_pos_.x + size_.x * EXEC_LT_POS.x,
                          screen_pos_.y + size_.y * EXEC_LT_POS.y};
  geom::vect2_t lt_sz = {size_.x * EXEC_LT_SZ.x, size_.y * EXEC_LT_SZ.y};

  cairo_utils::draw_rect(cr, lt_pos, lt_sz, EXEC_LT_CLR);
}
} // namespace fms_displays