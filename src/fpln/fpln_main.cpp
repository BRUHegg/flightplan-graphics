#include "fpln_main.hpp"

#include <cstddef>
#include <libnav/arpt_db.hpp>
#include <libnav/awy_db.hpp>
#include <libnav/navaid_db.hpp>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <util/pathlib.hpp>

#include "fpln_base.hpp"
#include "fpln_sys.hpp"
#include <util/util.hpp>

namespace fms_core {

FlightPlan::FlightPlan(util::OpaquePointer<libnav::ArptDB> apt_db,
                       util::OpaquePointer<libnav::NavaidDB> nav_db,
                       util::OpaquePointer<libnav::AwyDB> aw_db,
                       pathlib::Path cifp_path)
    : fpln_{apt_db, nav_db, aw_db, cifp_path} {}

double FlightPlan::get_id() const noexcept {
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, get_id, main_mutex_)}

std::size_t FlightPlan::get_leg_list_sz() const noexcept {
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, get_leg_list_sz,
                               main_mutex_)}

std::size_t FlightPlan::get_seg_list_sz() const noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, get_seg_list_sz,
                             main_mutex_)
}

double FlightPlan::get_ll_seg(
    std::size_t start, std::size_t l,
    std::vector<list_node_ref_t<leg_list_data_t>>* out,
    int* act_idx_out) noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, get_ll_seg, main_mutex_,
                             start, l, out, act_idx_out)
}

double FlightPlan::get_sl_seg(
    std::size_t start, std::size_t l,
    std::vector<list_node_ref_t<fpl_seg_t>>* out) noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, get_sl_seg, main_mutex_,
                             start, l, out)
}

bool FlightPlan::is_active() const noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, is_active, main_mutex_)
}

bool FlightPlan::can_activate() const noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, can_activate, main_mutex_)
}

void FlightPlan::activate() {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, activate, main_mutex_)
}

void FlightPlan::deactivate() {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, deactivate, main_mutex_)
}

void FlightPlan::print_refs() const noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FlightPlanBase, print_refs, main_mutex_)
}

void FlightPlan::copy_from_other(FlightPlan& other){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, copy_from_other, main_mutex_,
                               other.fpln_)}

libnav::DbErr FlightPlan::load_from_fms(const std::string& file_nm,
                                        bool set_arpts) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, load_from_fms, main_mutex_,
                             file_nm, set_arpts)
}

void FlightPlan::save_to_fms(const std::string& file_nm,
                             bool save_sid_star) const {
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, save_to_fms, main_mutex_,
                               file_nm, save_sid_star)}

std::string FlightPlan::get_co_rte_nm() const noexcept {
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_co_rte_nm, main_mutex_)}

libnav::DbErr FlightPlan::set_dep(std::string icao){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_dep, main_mutex_,
                               icao)}

std::string FlightPlan::get_dep_icao() const noexcept {
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_dep_icao, main_mutex_)}

libnav::DbErr FlightPlan::set_arr(std::string icao){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_arr, main_mutex_,
                               icao)}

std::string FlightPlan::get_arr_icao(){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_arr_icao, main_mutex_)}

// Runway functions:

std::vector<std::string> FlightPlan::get_dep_rwys(bool filter_rwy,
                                                  bool filter_sid)
    const noexcept {MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_dep_rwys,
                                               main_mutex_, filter_rwy, filter_sid)}

std::vector<std::string> FlightPlan::get_arr_rwys(
    bool filter_rwy, bool filter_star, bool is_arr) const noexcept {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_arr_rwys, main_mutex_,
                             filter_rwy, filter_star, is_arr)
}

bool FlightPlan::set_dep_rwy(const std::string& rwy){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_dep_rwy, main_mutex_,
                               rwy)}

std::string FlightPlan::get_dep_rwy() {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_dep_rwy, main_mutex_)
}

bool FlightPlan::get_dep_rwy_data(libnav::runway_entry_t* out) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_dep_rwy_data, main_mutex_,
                             out)
}

bool FlightPlan::set_arr_rwy(std::string& rwy){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_arr_rwy, main_mutex_,
                               rwy)}

std::string FlightPlan::get_arr_rwy() const {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_arr_rwy, main_mutex_)
}

bool FlightPlan::get_arr_rwy_data(libnav::runway_entry_t* out){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_arr_rwy_data, main_mutex_,
                               out)}

// Airport procedure functions:

std::string FlightPlan::get_curr_proc(ProcType tp, bool trans){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_curr_proc, main_mutex_,
                               tp, trans)}

std::vector<std::string> FlightPlan::get_arpt_proc(ProcType tp,
                                                   bool is_arr,
                                                   bool filter_rwy,
                                                   bool filter_proc){
    MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_arpt_proc, main_mutex_,
                               tp, is_arr, filter_rwy, filter_proc)}

std::vector<std::string> FlightPlan::get_arpt_proc_trans(
    ProcType tp, bool is_rwy, bool is_arr, bool incl_none) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, get_arpt_proc_trans, main_mutex_,
                             tp, is_rwy, is_arr, incl_none)
}

bool FlightPlan::set_arpt_proc(ProcType tp, std::string proc_nm,
                               bool is_arr) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_arpt_proc, main_mutex_,
                             tp, proc_nm, is_arr)
}

bool FlightPlan::set_arpt_proc_trans(ProcType tp, std::string trans,
                                     bool is_arr) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_arpt_proc_trans, main_mutex_,
                             tp, trans, is_arr)
}

// Enroute:

bool FlightPlan::add_enrt_seg(timed_ptr_t<seg_list_node_t> next,
                              std::string name) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, add_enrt_seg, main_mutex_,
                             next, name)
}

// End MUST be an airway id

bool FlightPlan::awy_insert_str(timed_ptr_t<seg_list_node_t> next,
                                std::string end_id) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, awy_insert_str, main_mutex_,
                             next, end_id)
}

bool FlightPlan::awy_insert(timed_ptr_t<seg_list_node_t> next,
                            libnav::waypoint_t end) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, awy_insert, main_mutex_,
                             next, end)
}

bool FlightPlan::delete_via(timed_ptr_t<seg_list_node_t> next) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, delete_via, main_mutex_,
                             next)
}

bool FlightPlan::delete_seg_end(timed_ptr_t<seg_list_node_t> next) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, delete_seg_end, main_mutex_,
                             next)
}

// Leg list interface functions:

bool FlightPlan::dir_from_to(timed_ptr_t<leg_list_node_t> from,
                             timed_ptr_t<leg_list_node_t> to) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, dir_from_to, main_mutex_,
                             from, to)
}

void FlightPlan::add_direct(libnav::waypoint_t wpt,
                            timed_ptr_t<leg_list_node_t> next) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, add_direct, main_mutex_,
                             wpt, next)
}

bool FlightPlan::delete_leg(timed_ptr_t<leg_list_node_t> next) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, delete_leg, main_mutex_,
                             next)
}

void FlightPlan::set_spd_cstr(timed_ptr_t<leg_list_node_t> node,
                              spd_cstr_t cst) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_spd_cstr, main_mutex_,
                             node, cst)
}

// This one only sets alt1 for now.

void FlightPlan::set_alt_cstr(timed_ptr_t<leg_list_node_t> node,
                              alt_cstr_t cst) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, set_alt_cstr, main_mutex_,
                             node, cst)
}

// Calculation function

void FlightPlan::update(double hdg_trk_diff) {
  MY_MUTEX_WRAPPER_FUNC_BODY(fpln_, FplnInt, update, main_mutex_,
                             hdg_trk_diff)
}
}  // namespace fms_core
