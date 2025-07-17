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
#include <util.hpp>
#include <memory>


namespace StratosphereAvionics
{
    constexpr size_t N_LEG_PROJ_CACHE_SZ = 200;
    constexpr size_t N_LN_JOINT_CACHE_SZ = N_LEG_PROJ_CACHE_SZ - 1;
    constexpr size_t N_PROJ_CACHE_SZ = 202;
    constexpr size_t DEP_RWY_PROJ_IDX = N_PROJ_CACHE_SZ-2;
    constexpr size_t ARR_RWY_PROJ_IDX = N_PROJ_CACHE_SZ-1;
    constexpr size_t N_ND_SDS = test::N_INTFCS; // Essentially this is how many NDs we can have
    constexpr size_t N_MP_DATA_SZ = N_ND_SDS*test::N_FPL_SYS_RTES;
    constexpr double N_MAX_DIST_NM = 600;
    constexpr double ND_DEFAULT_RNG_NM = 10;
    // Percentage of resolution that translates into full range:
    static std::unordered_map<test::NDMode, double, util::enum_class_hash_t> ND_RNG_FULL_RES_COEFF = {
        {test::NDMode::MAP, 0.76},
        {test::NDMode::PLAN, 0.4}
    };
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
    // For flightplan line dashes:
    const double FPLN_LN_DASH[] = {9.0, 9.0};
    constexpr int N_FPLN_DASHES  = sizeof(FPLN_LN_DASH) / sizeof(FPLN_LN_DASH[0]);
    constexpr int V_RTE_NOT_DRAWN = -1;


    constexpr double ND_WPT_FONT_SZ = 23;
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
    constexpr double ND_FPL_LINE_THICK = 0.0042;
    constexpr double ND_PRJ_CTR_V_OFFS_PLAN = 0.01; // Offset from the display center
    constexpr double ND_PRJ_CTR_V_OFFS_MAP = 0.345; // Offset from the display center
    // Route drawing
    constexpr geom::vect2_t FIX_NAME_OFFS = {0.02, 0.03};
    const std::vector<geom::vect3_t> ND_RTE_CLRS = {cairo_utils::WHITE, 
        cairo_utils::MAGENTA, cairo_utils::ND_CYAN};
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
    // General:
    constexpr geom::vect2_t MAP_HDG_OFFS = {0, -0.049};

    constexpr geom::vect3_t ND_BCKGRND_CLR = cairo_utils::BLACK;


    const std::string WPT_INACT_NAME = "wpt_inact";
    const std::string WPT_ACT_NAME = "wpt_act";
    const std::string AIRPLANE_NAME = "airplane";
    const std::string PLN_BACKGND_INNER_NAME = "pln_back_inner";
    const std::string PLN_BACKGND_OUTER_NAME = "pln_back_outer";
    const std::string MAP_BACKGND_NAME = "map_back";
    const std::string MAP_HDG_NAME = "map_hdg";
    const std::string MAP_AC_TRI  = "map_ac_ico";

    const std::vector<double> ND_RANGES_NM = {10, 20, 40, 80, 160, 320, 640};
    // Only the supported modes are in ND_MDS
    const std::vector<test::NDMode> ND_MDS = {test::NDMode::MAP, test::NDMode::PLAN};
    constexpr double RNG_DEC_1_NM = 2.5;
    constexpr double RNG_DEC_2_NM = 1.25;


    struct nd_util_idx_t
    {
        size_t sd_idx, dt_idx;
    };

    struct leg_proj_t
    {
        geom::vect2_t start, end, arc_ctr, end_wpt;
        bool is_arc, is_finite, is_rwy, has_path;
        double turn_rad_nm;
        std::string end_nm;
        geom::line_joint_t *joint;


        std::string get_draw_nm();
    };

    struct map_data_t
    {
        leg_proj_t *proj_legs;
        geom::line_joint_t *line_joints;

        size_t n_act_proj_legs;
        size_t n_act_joints;


        void create();

        void destroy();
    };


    class NDData
    {
    public:
        NDData(std::shared_ptr<test::FPLSys> fpl_sys);

        void set_mode(size_t sd_idx, test::NDMode md, bool set_ctr=false);

        std::pair<test::NDMode, bool> get_mode(size_t sd_idx) const;

        std::vector<int> get_rte_draw_seq(size_t sd_idx);

        size_t get_proj_legs(leg_proj_t **out, size_t sd_idx, size_t dt_idx);

        int get_act_leg_idx(size_t sd_idx);

        bool get_ac_pos(geom::vect2_t *out, size_t sd_idx);

        double get_hdg_trk(size_t sd_idx) const;

        test::hdg_info_t get_hdg_data();

        test::spd_info_t get_spd_data();

        test::act_leg_info_t get_act_leg_info();

        bool has_dep_rwy(size_t idx);

        bool has_arr_rwy(size_t idx);

        void switch_range(bool down, size_t sd_idx);

        double get_range(size_t sd_idx);

        void update();

        ~NDData();

    private:
        std::vector<std::shared_ptr<test::FplnInt>> m_fpl_vec;
        std::shared_ptr<test::FPLSys> m_fpl_sys_ptr;

        // Of size N_FPL_SYS_RTES
        std::vector<test::nd_leg_data_t*> m_leg_data;
        std::vector<size_t> m_leg_data_sz;

        std::vector<double> m_fpl_id_last;
        std::vector<bool> m_has_dep_rwy, m_has_arr_rwy;
        std::vector<int> m_act_leg_idx;
        // -1 if route is not to be drawn, index otherwise.
        // Routes are ordered by color. Refer to ND_RTE_CLRS.
        std::vector<std::vector<int>> m_rte_draw_seq; 

        // 2*number of routes
        std::vector<map_data_t> m_mp_data;
       
        std::vector<int> m_act_leg_idx_sd;
        // Stored 1 per fo, 1 per cap
        std::vector<std::pair<test::NDMode, bool>> cr_mds;
        std::vector<geom::vect2_t> m_ac_pos_proj;
        std::vector<bool> m_ac_pos_ok;
        std::vector<size_t> m_rng_idx;
        std::vector<geo::point> m_ctr;

        test::hdg_info_t m_hdg_data;


        static bool bound_check(double x1, double x2, double rng);

        static nd_util_idx_t get_util_idx(size_t gn_idx);

        double get_cr_rot(size_t sd_idx) const;

        std::pair<geo::point, double> get_proj_params(size_t sd_idx) const;

        bool in_view(geom::vect2_t start, geom::vect2_t end, size_t sd_idx);

        void update_rte_draw_seq();

        void update_ctr(size_t sd_idx);

        bool project_ac_pos(size_t sd_idx);

        void project_legs(size_t gn_idx);

        void project_rwys(size_t gn_idx);

        void fetch_legs(size_t dt_idx);

        void update_fpl(size_t dt_idx);
    };

    class NDDisplay
    {
    public:
        NDDisplay(std::shared_ptr<NDData> data, 
            std::shared_ptr<cairo_utils::texture_manager_t> mngr,
            cairo_font_face_t *ff, geom::vect2_t pos, geom::vect2_t sz, size_t sd_idx);

        void draw(cairo_t *cr);

    private:
        std::shared_ptr<NDData> nd_data;
        std::shared_ptr<cairo_utils::texture_manager_t> tex_mngr;

        test::NDMode cr_md;
        bool is_ctr;

        cairo_font_face_t* font_face;

        geom::vect2_t scr_pos;
        geom::vect2_t size;
        geom::vect2_t map_ctr, scale_factor;
        test::hdg_info_t hdg_data;
        double rng, curr_rng;

        size_t side_idx;


        void update_mode();

        void update_map_params();

        geom::vect2_t get_screen_coords(geom::vect2_t src);

        void draw_line_joint(cairo_t *cr, geom::line_joint_t lj, geom::vect3_t ln_clr);

        void draw_flight_plan(cairo_t *cr, bool draw_labels, geom::vect3_t ln_clr, 
            size_t idx=0);

        void draw_ext_rwy_ctr_line(cairo_t *cr, leg_proj_t rnw_proj);

        void draw_runway(cairo_t *cr, leg_proj_t rnw_proj);

        void draw_runways(cairo_t *cr, size_t idx=0);

        void draw_all_fplns(cairo_t *cr);

        void draw_airplane(cairo_t *cr);

        void draw_background(cairo_t *cr, bool draw_inner);

        void draw_act_leg_info(cairo_t *cr);

        void draw_spd_info(cairo_t *cr);

        void draw_range(cairo_t *cr);
    };
} // namespace StratosphereAvionics
