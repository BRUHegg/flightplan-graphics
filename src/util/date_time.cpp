#include "date_time.hpp"

#include <chrono>

namespace {

std::chrono::year_month_day AIRAC_REF_DATE{std::chrono::year{2024},
    std::chrono::month{1}, std::chrono::day{25}};
unsigned BASE_AIRAC = 2401;
unsigned CNT_AIRAC_DAYS = 28;
unsigned CNT_AIRAC_PER_YEAR = 365 / CNT_AIRAC_DAYS;
} // namespace

namespace util {

std::chrono::month_day ymd_to_md(std::chrono::year_month_day ymd) {
  return std::chrono::month_day{ymd.month(), ymd.day()};
}

airac_dates_t get_airac_dates(unsigned airac) {
  airac_dates_t res;
  int diff_year = (airac - BASE_AIRAC) / 100;
  int diff_cycle = diff_year * CNT_AIRAC_PER_YEAR + 
    ((airac % 100) - (BASE_AIRAC % 100));
  res.base = std::chrono::sys_days{AIRAC_REF_DATE} + 
    std::chrono::days{CNT_AIRAC_DAYS * diff_cycle};
  res.expiry = std::chrono::sys_days{res.base} + 
    std::chrono::days{CNT_AIRAC_DAYS};
  return res;
}
} // namespace util
