/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This source file contains declarations of classes, functions, etc 
	used in the ND implementation. Author: discord/bruh4096#4512
*/

#include <fpln/fpln_sys.hpp>
#include <libnav/str_utils.hpp>
#include <common/cairo_utils.hpp>
#include <geom.hpp>
#include <memory>


namespace StratosphereAvionics
{
    constexpr size_t N_LEG_PROJ_CACHE_SZ = 200;
    constexpr size_t N_LN_JOINT_CACHE_SZ = N_LEG_PROJ_CACHE_SZ - 1;
    constexpr size_t N_PROJ_CACHE_SZ = 202;
    constexpr size_t DEP_RWY_PROJ_IDX = N_PROJ_CACHE_SZ-2;
    constexpr size_t ARR_RWY_PROJ_IDX = N_PROJ_CACHE_SZ-1;
    constexpr double N_MAX_DIST_NM = 600;
    constexpr double ND_DEFAULT_RNG_NM = 10;
    // Percentage of resolution that translates into full range:
    constexpr double ND_RNG_FULL_RES_COEFF = 0.4;
    // Percentage of horisontal resolution that translates into runway width
    constexpr double DEFAULT_RWY_WIDTH = 0.015;
    // Percentage of horisontal resolution that translates into thikness of runway 
    // side line
    constexpr double RWY_SIDE_THICK = 0.0025;
    // Length of extended runway center line in nautical miles(one direction)
    constexpr double N_RWY_EXT_CTR_LINE_NM = 16;
    // For extended centerline dashes:
    const double RWY_EXT_CTR_LINE_DASH[] = {9.0, 9.0};
    constexpr int N_RWY_DASHES  = sizeof(RWY_EXT_CTR_LINE_DASH) / sizeof(RWY_EXT_CTR_LINE_DASH[0]);


    constexpr double ND_WPT_FONT_SZ = 18;
    constexpr double ND_ACT_INFO_MAIN_FONT_SZ = 21;
    constexpr double ND_ACT_INFO_DIST_FONT_SZ = 19;
    constexpr double ND_SPD_BIG_FONT_SZ = 26;
    constexpr double ND_SPD_SMALL_FONT_SZ = 21;
    constexpr double GS_THRESH_BIG_KTS = 30;
    constexpr double TAS_DISPL_THRESH_KTS = 100;
    // Inverse of percentage of resolution that contributes to the scaling factor of waypoint image
    constexpr double WPT_SCALE_FACT = 900;
    // Percentage of horizontal resolution that translates into radius of pseudo waypoint label
    constexpr double PSEUDO_WPT_RADIUS_RAT = 0.007;
    constexpr double PSEUDO_WPT_THICK_RAT = 0.0025;
    // Percentage of horisontal resolution that translates into magenta line width
    constexpr double ND_FPL_LINE_THICK = 0.003;
    // Route drawing
    constexpr geom::vect2_t FIX_NAME_OFFS = {0.02, 0.03};
    // Active leg
    constexpr geom::vect2_t ACT_LEG_NAME_OFFS = {0.911, 0.03};
    constexpr geom::vect2_t ACT_LEG_TIME_OFFS = {0.911, 0.05};
    constexpr geom::vect2_t ACT_LEG_DIST_OFFS = {0.911, 0.07};
    constexpr geom::vect2_t ACT_LEG_NM_OFFS = {0.96, 0.07};
    // Speed
    constexpr geom::vect2_t GS_OFFS = {0.061, 0.034};
    constexpr geom::vect2_t GS_TEXT_OFFS = {0.003, 0.034};
    constexpr geom::vect2_t TAS_OFFS = {0.15, 0.034};
    constexpr geom::vect2_t TAS_TEXT_OFFS = {0.079, 0.034};


    const std::string WPT_INACT_NAME = "wpt_inact";
    const std::string WPT_ACT_NAME = "wpt_act";
    const std::string AIRPLANE_NAME = "airplane";
    const std::string PLN_BACKGND_INNER_NAME = "pln_back_inner";
    const std::string PLN_BACKGND_OUTER_NAME = "pln_back_outer";

    const std::vector<double> ND_RANGES_NM = {10, 20, 40, 80, 160, 320, 640};
    constexpr double RNG_DEC_1_NM = 2.5;
    constexpr double RNG_DEC_2_NM = 1.25;


    struct leg_proj_t
    {
        geom::vect2_t start, end, arc_ctr, end_wpt;
        bool is_arc, is_finite, is_rwy, has_path;
        double turn_rad_nm;
        std::string end_nm;
        geom::line_joint_t *joint;


        std::string get_draw_nm();
    };


    class NDData
    {
    public:
        NDData(std::shared_ptr<test::FPLSys> fpl_sys);

        size_t get_proj_legs(leg_proj_t **out, bool fo_side);

        int get_act_leg_idx(bool fo_side);

        bool get_ac_pos(geom::vect2_t *out, bool fo_side);

        test::hdg_info_t get_hdg_data();

        test::spd_info_t get_spd_data();

        test::act_leg_info_t get_act_leg_info();

        bool has_dep_rwy();

        bool has_arr_rwy();

        void switch_range(bool down, bool fo_side);

        double get_range(bool fo_side);

        void update();

        ~NDData();

    private:
        std::shared_ptr<test::FplnInt> m_fpl_ptr;
        std::shared_ptr<test::FPLSys> m_fpl_sys_ptr;

        test::nd_leg_data_t *m_leg_data;
        size_t m_n_act_leg_data;

        leg_proj_t *m_proj_legs_cap;
        leg_proj_t *m_proj_legs_fo;
        geom::line_joint_t *m_line_joints_cap;
        geom::line_joint_t *m_line_joints_fo;

        geom::vect2_t m_ac_pos_proj_cap;
        geom::vect2_t m_ac_pos_proj_fo;

        size_t m_n_act_proj_legs_cap;
        size_t m_n_act_proj_legs_fo;
        size_t m_n_act_joints_cap;
        size_t m_n_act_joints_fo;

        geo::point m_ctr_cap;
        geo::point m_ctr_fo;

        test::hdg_info_t m_hdg_data;

        size_t m_rng_idx_cap;
        size_t m_rng_idx_fo;

        double m_fpl_id_last;

        bool m_has_dep_rwy, m_has_arr_rwy, m_ac_pos_ok_cap, m_ac_pos_ok_fo;

        int m_act_leg_idx, m_act_leg_idx_cap, m_act_leg_idx_fo;


        static bool bound_check(double x1, double x2, double rng);

        bool in_view(geom::vect2_t start, geom::vect2_t , bool fo_side);

        void update_ctr(geo::point *ctr, bool fo_side);

        void project_legs(bool fo_side);

        void project_rwys(bool fo_side);

        bool project_ac_pos(bool fo_side);

        void fetch_legs();
    };

    class NDDisplay
    {
    public:
        NDDisplay(std::shared_ptr<NDData> data, 
            std::shared_ptr<cairo_utils::texture_manager_t> mngr,
            cairo_font_face_t *ff, geom::vect2_t pos, geom::vect2_t sz, bool fo_sd);

        void draw(cairo_t *cr);

    private:
        std::shared_ptr<NDData> nd_data;
        std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;

        cairo_font_face_t* font_face;

        geom::vect2_t scr_pos;
        geom::vect2_t size;
        geom::vect2_t map_ctr, scale_factor;
        test::hdg_info_t hdg_data;
        double rng, curr_rng;

        bool fo_side;


        void update_map_params();

        geom::vect2_t get_screen_coords(geom::vect2_t src);

        void draw_line_joint(cairo_t *cr, geom::line_joint_t lj);

        void draw_flight_plan(cairo_t *cr, bool draw_labels);

        void draw_ext_rwy_ctr_line(cairo_t *cr, leg_proj_t rnw_proj);

        void draw_runway(cairo_t *cr, leg_proj_t rnw_proj);

        void draw_runways(cairo_t *cr);

        void draw_airplane(cairo_t *cr);

        void draw_background(cairo_t *cr, bool draw_inner);

        void draw_act_leg_info(cairo_t *cr);

        void draw_spd_info(cairo_t *cr);

        void draw_range(cairo_t *cr);
    };
} // namespace StratosphereAvionics
