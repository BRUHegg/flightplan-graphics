/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This source file contains declarations of classes, functions, etc 
	used in the ND implementation. Author: discord/bruh4096#4512
*/

#include <fpln/fpln_sys.hpp>
#include <geom.hpp>
#include <memory>


namespace StratosphereAvionics
{
    constexpr size_t N_LEG_PROJ_CACHE_SZ = 200;
    constexpr double N_MAX_DIST_NM = 600;


    struct leg_proj_t
    {
        geom::vect2_t start, end, arc_ctr;
        bool is_arc, is_finite;
        double turn_rad_nm;
        std::string end_nm;
    };


    class NDData
    {
    public:
        NDData(std::shared_ptr<test::FPLSys> fpl_sys);

        size_t get_proj_legs(leg_proj_t **out, bool fo_side);

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

        double m_fpl_id_last;


        void update_ctr(geo::point *ctr, bool fo_side);

        void project_legs(bool fo_side);

        void fetch_legs();
    };
} // namespace StratosphereAvionics
