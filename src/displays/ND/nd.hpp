/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This source file contains declarations of classes, functions, etc 
	used in the ND implementation. Author: discord/bruh4096#4512
*/

#include <fpln/fpln_sys.hpp>
#include <common/cairo_utils.hpp>
#include <geom.hpp>
#include <memory>


namespace StratosphereAvionics
{
    constexpr size_t N_LEG_PROJ_CACHE_SZ = 200;
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


    constexpr double ND_WPT_FONT_SZ = 18;
    // Percentage of horisontal resolution that translates into magenta line width
    constexpr double ND_FPL_LINE_THICK = 0.003;
    constexpr geom::vect2_t FIX_NAME_OFFS = {0.03, 0.05};

    const std::string WPT_INACT_NAME = "wpt_inact";
    const std::string WPT_ACT_NAME = "wpt_act";

    const std::vector<double> ND_RANGES_NM = {10, 20, 40, 80, 160, 320, 640};


    struct leg_proj_t
    {
        geom::vect2_t start, end, arc_ctr, end_wpt;
        bool is_arc, is_finite, is_rwy;
        double turn_rad_nm;
        std::string end_nm;
    };


    geom::vect2_t get_projection(double brng_rad, double dist_nm);

    geom::vect2_t project_point(geo::point tgt, geo::point p_ctr);


    class NDData
    {
    public:
        NDData(std::shared_ptr<test::FPLSys> fpl_sys);

        size_t get_proj_legs(leg_proj_t **out, bool fo_side);

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
        size_t m_n_act_proj_legs_cap;
        size_t m_n_act_proj_legs_fo;

        geo::point m_ctr_cap;
        geo::point m_ctr_fo;

        size_t m_rng_idx_cap;
        size_t m_rng_idx_fo;

        double m_fpl_id_last;

        bool m_has_dep_rwy, m_has_arr_rwy;


        bool in_view(geom::vect2_t start, geom::vect2_t , bool fo_side);

        void update_ctr(geo::point *ctr, bool fo_side);

        void project_legs(bool fo_side);

        void project_rwys(bool fo_side);

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
        double rng;

        bool fo_side;


        geom::vect2_t get_screen_coords(geom::vect2_t src, geom::vect2_t map_ctr, 
            geom::vect2_t sc);

        void draw_flight_plan(cairo_t *cr, bool draw_labels);

        void draw_runway(cairo_t *cr, leg_proj_t rnw_proj);

        void draw_runways(cairo_t *cr);
    };
} // namespace StratosphereAvionics
