#pragma once

#include <string>
#include <vector>

#include <displays/CDU/common.hpp>
#include <libnav/geo_utils.hpp>

namespace cdu_pages {

std::string string_from_error(fms_displays::CDUError err) noexcept;

bool scratchpad_has_delete(const std::string& scratchpad) noexcept;

std::string get_scratchpad_pos(geo::point pos) noexcept;

std::string str_align_right(const std::string& str);

std::string get_displayed_pos(geo::point pos) noexcept;

struct cdu_scr_data_t {
  std::string heading_big, heading_small;
  fms_displays::CDUColor heading_color;
  std::vector<std::string> data_lines;
  std::vector<std::string> chr_sts;

  cdu_scr_data_t();
};

struct cdu_event_res_t {
  fms_displays::CDUError err;
  fms_displays::CDUPage page;
};

class PageBase {
public:
  virtual fms_displays::CDUPage get_page_number() const noexcept = 0;

  virtual void update() noexcept = 0;

  virtual cdu_event_res_t on_event(fms_displays::cdu_event_type event, 
    const std::string& scratchpad, std::string& s_out) noexcept = 0;

  virtual cdu_scr_data_t get_screen_data() const noexcept = 0;

  virtual ~PageBase() = default;
};
} // namespace cdu_pages
