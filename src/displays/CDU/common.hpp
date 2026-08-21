#pragma once

namespace fms_displays {

constexpr char DELETE_SYMBOL = 'd';
constexpr unsigned N_CDU_DATA_LINES = 6;
constexpr int N_CDU_DATA_COLS = 24;

enum class CDUColor { WHITE, GREEN, CYAN, MAGENTA };

enum class CDUError { 
  NONE,
  NOT_IN_DATABASE, 
  INVALID_ENTRY,
  INVALID_DELETE,
  INVALID_ROUTE_UPLINK
};

enum class CDUPage {
  RTE,
  DEP_ARR_INTRO,
  DEP1,
  ARR1,
  DEP2,
  ARR2,
  LEGS,
  INIT_REF,
  ALTN,
  VNAV,
  FIX,
  HOLD,
  FMC_COMM,
  PROG,
  MENU,
  IDENT,
  POS_INIT,
  INIT_REF_INDEX,
  NAV_RAD,
  PREV_PAGE,
  NEXT_PAGE
};

using cdu_event_type = int;
constexpr cdu_event_type CDU_KEY_LSK_TOP = 1;
constexpr cdu_event_type CDU_KEY_RSK_TOP = 7;
constexpr cdu_event_type CDU_KEY_INIT_REF = 13;
constexpr cdu_event_type CDU_KEY_A = 27;
constexpr cdu_event_type CDU_KEY_SP = 53;
constexpr cdu_event_type CDU_KEY_DELETE = 54;
constexpr cdu_event_type CDU_KEY_SLASH = 55;
constexpr cdu_event_type CDU_KEY_CLR = 56;
constexpr cdu_event_type CDU_KEY_1 = 57;
constexpr cdu_event_type CDU_KEY_DOT = 66;
constexpr cdu_event_type CDU_KEY_0 = 67;
constexpr cdu_event_type CDU_KEY_PM = 68;  // +/- key
constexpr cdu_event_type CDU_KEY_EXEC = 69;

// CDU char states:
constexpr char CDU_S_WHITE = 'w';
constexpr char CDU_B_WHITE = 'W';
constexpr char CDU_S_CYAN = 'c';
constexpr char CDU_B_CYAN = 'C';
constexpr char CDU_S_GREEN = 'g';
constexpr char CDU_B_GREEN = 'G';
constexpr char CDU_S_MAGENTA = 'm';
constexpr char CDU_B_MAGENTA = 'M';

const std::string ALL_DASH = std::string(N_CDU_DATA_COLS, '-');
} // namespace fms_displays
