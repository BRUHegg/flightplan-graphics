#include "cdu_widget.hpp"

#include <chrono>
#include <memory>

#include <cairo/cairo.h>

#include <common/bytemap.hpp>
#include <common/cairo_utils.hpp>
#include <util/geom.hpp>

namespace {

constexpr double CDU_PRS_INTV_SEC = 0.001;  // Threshold to skip gtk's double-clicks

constexpr geom::vect2_t EXEC_LT_POS = {0.793, 0.582 * 0.964};
constexpr geom::vect2_t EXEC_LT_SZ = {0.08 * 1.2 * 0.75,
                                      0.7 * 0.08 * 1.08 * 0.37 * 0.5};
constexpr geom::vect3_t EXEC_LT_CLR = {0.955, 0.906, 0.269};
} // namespace

namespace fms_displays {

CDUWidget::CDUWidget(geom::vect2_t pos, geom::vect2_t sz, 
             std::shared_ptr<cairo_utils::texture_manager_t> tm,
             std::shared_ptr<CDU> cdu, std::shared_ptr<CDUDisplay> cdu_displ, 
             byteutils::Bytemap* bm) : tex_mngr_{tm}, cdu_displ_{cdu_displ},
             cdu_ptr_{cdu}, key_map_{bm}, screen_pos_{pos}, size_{sz} {
  texture_size_ = cairo_utils::get_surf_sz(tex_mngr_->data[CDU_TEXTURE_NAME]);
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
  cairo_utils::draw_image(cr, tex_mngr_->data[CDU_TEXTURE_NAME], screen_pos_,
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