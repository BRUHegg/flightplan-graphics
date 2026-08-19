#pragma once

#include <chrono>

namespace util {

std::chrono::month_day ymd_to_md(std::chrono::year_month_day ymd);

struct airac_dates_t {
  std::chrono::year_month_day base;
  std::chrono::year_month_day expiry;
};

airac_dates_t get_airac_dates(unsigned airac);
} // namespace util
