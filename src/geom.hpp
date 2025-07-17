/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	This header file contains definitions of utility functions for geometry. 
    Mostly vectors. Functions for azimuth equidistant are also included.
    Author: discord/bruh4096#4512(Tim G.)
*/


#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <libnav/geo_utils.hpp>
#include <assert.h>


namespace geom
{
    enum class JointType
    {
        CIRC,
        CIRC_CIRC,
        LINE
    };

    constexpr double DEG_TO_RAD = M_PI / 180.0;
	constexpr double RAD_TO_DEG = 180.0 / M_PI;


    struct vect2_t
    {
        double x, y;


        double absval()
        {
            return sqrt(x * x + y * y);
        }

        vect2_t get_unit()
        {
            double av = absval();
            if(av == 0)
                return {};
            return {x / av, y / av};
        }

        double dist_to(vect2_t other)
        {
            return sqrt((x-other.x) * (x-other.x) + (y - other.y) * (y - other.y));
        }

        double cross_prod(vect2_t other)
        {
            return x * other.y - y * other.x;
        }

        double dot_prod(vect2_t other)
        {
            return x * other.x + y * other.y;
        }

        vect2_t scmul(double num)
        {
            return {x * num, y * num};
        }

        vect2_t scdiv(double num)
        {
            if(num == 0)
                return {};
            return {x / num, y / num};
        }

        vect2_t operator+(vect2_t const& other)
        {
            return {x + other.x, y + other.y};
        }

        vect2_t operator-(vect2_t const& other)
        {
            return {x - other.x, y - other.y};
        }

        vect2_t operator*(vect2_t const& other)
        {
            return {x * other.x, y * other.y};
        }

        vect2_t operator/(vect2_t const& other)
        {
            return {x / other.x, y / other.y};
        }
    };

    struct vect3_t
    {   
        double x, y, z;


        double absval()
        {
            return sqrt(x * x + y * y + z * z);
        }

        vect3_t scmul(double num)
        {
            return {x * num, y * num, z * num};
        }

        vect3_t operator+(vect3_t const& other)
        {
            return {x + other.x, y + other.y, z + other.z};
        }

        vect3_t operator-(vect3_t const& other)
        {
            return {x - other.x, y - other.y, z - other.z};
        }

        vect3_t operator*(vect3_t const& other)
        {
            return {x * other.x, y * other.y, z * other.z};
        }
    };

    struct arc_t
    {
        double ang_start_rad, ang_end_rad;
        vect2_t pos;
    };

    struct line_t
    {
        vect2_t start, end;
    };

    struct line_joint_t
    {
        JointType tp;
        arc_t arc1, arc2;
        line_t line;
        double turn_radius;
    };


    inline bool operator==(vect2_t const& a, vect2_t const& b)
    {
        return a.x == b.x && a.y == b.y;
    }

    inline bool operator!=(vect2_t const& a, vect2_t const& b)
    {
        return !operator==(a, b);
    }

    inline bool operator==(vect3_t const& a, vect3_t const& b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }

    inline bool operator!=(vect3_t const& a, vect3_t const& b)
    {
        return !operator==(a, b);
    }

    /*
        Function: get_point_line_dist
        Description:
        Gets distance from point to line.
        @param x: point to measure distance to
        @param la: point on line
        @param lb: other point on line. NOT x
        @return distance
    */

    inline double get_point_line_dist(vect2_t x, vect2_t la, vect2_t lb)
    {
        double s_a = x.dist_to(la);
        double s_b = x.dist_to(lb);
        double s_c = la.dist_to(lb);

        assert(s_c != 0);

        double p = (s_a + s_b + s_c) / 2;
        double area = sqrt(p * (p - s_a) * (p - s_b) * (p - s_c));
        return area * 2 / s_c;
    }

    inline double get_vec_to_line(vect2_t px, vect2_t pl1, vect2_t pl2, vect2_t *out)
    {
        vect2_t l_vec = pl2 - pl1;
        double abs_lvec = l_vec.absval();
        assert(abs_lvec != 0);

        vect2_t a = pl1 - px;
        vect2_t b = pl2 - px;

        vect2_t nml = {l_vec.y, -l_vec.x};
        if(nml.dot_prod(a) < 0)
            nml = nml.scmul(-1);
        
        double dist = abs(a.cross_prod(b)) / abs_lvec;

        *out = nml.get_unit();
        return dist;
    }

    /*
        Function: get_vect_circ_isect_dist
        Description:
        Gets distance from point intercection of vector and circle.
        @param ps: point from which the vector starts
        @param po: center of the circle
        @param a: unit vector that points from ps
        @param c_r: radius of the circle
        @return distance. Positive if there is an intersection. -1 otherwise.
    */

    inline double get_vect_circ_isect_dist(vect2_t ps, vect2_t po, vect2_t a, double c_r)
    {
        vect2_t b = po - ps;

        double abs_b = b.absval();
        if(abs_b == c_r)
            return 0;

        double l = abs(a.cross_prod(b));

        if(l > c_r)
        {
            return -1;
        }
        else
        {
            double pq = sqrt(c_r * c_r - l * l);
            double ps = sqrt(abs_b * abs_b - l * l);

            if(abs_b > c_r)
            {
                return ps-pq;
            }
            else
            {
                double dp = a.dot_prod(b);

                if(dp < 0)
                {
                    return pq-ps;
                }
                else
                {
                    return pq+ps;
                }
            }
        }
    }

    inline double get_turn_isect_smpl(vect2_t pa, vect2_t pb, vect2_t ps, vect2_t a,
        vect2_t *out)
    {
        vect2_t b = pb - pa;
        b = b.get_unit();
        vect2_t tmp = ps - pa;
        double c = b.cross_prod(tmp);

        vect2_t n_b; // b's normal vector

        if(c < 0)
            n_b = {b.y, -b.x};
        else
            n_b = {-b.y, b.x};

        vect2_t ps2 = ps + a;
        double ln_dist = get_point_line_dist(pb, ps, ps2);
        
        double n_b_a_dot = n_b.dot_prod(a);
        
        if(n_b_a_dot == 0)  // n_b is perpendicular to a -> a||b or on same line
        {
            *out = pb + n_b.scmul(ln_dist);
            return ln_dist / 2;
        }
        else
        {
            double n_b_a_cross = n_b.cross_prod(a);
            if(n_b_a_cross == 0) // n_b and a are on 1 line. Not much can be done here
            {
                *out = pb;
                return 0;
            }

            double tan_alpha = n_b_a_cross / n_b_a_dot;
            double cos_alpha = n_b_a_dot / (n_b.absval() * a.absval());

            double ih = ln_dist / tan_alpha;
            double bi = abs(ih / cos_alpha);
            double r = (ln_dist * bi) / (ln_dist + bi);
            double im = (bi - r) * cos_alpha;

            vect2_t pi = pb + n_b.scmul(bi);
            *out = pi + a.scmul(-im);
            return r;
        }
    }

    inline double get_turn_rad(double ang_start_rad, double ang_end_rad, bool left_turn)
    {
        double diff = ang_start_rad - ang_end_rad;

        if(diff > 0 && left_turn)
        {
            return diff - 2 * M_PI;
        }
        else if(diff < 0 && !left_turn)
        {
            return diff + 2 * M_PI;
        }

        return diff;
    }

    inline arc_t get_circ_from_pts(vect2_t pos, vect2_t start, vect2_t end, 
        bool left_turn)
    {
        vect2_t v_start = start - pos;
        vect2_t v_end = end - pos;

        double ang_start_rad = atan2(v_start.y, v_start.x);
        double ang_end_rad = atan2(v_end.y, v_end.x);
        assert(!isnan(ang_start_rad) && !isnan(ang_end_rad));

        if(ang_start_rad < 0)
            ang_start_rad += 2 * M_PI;
        if(ang_end_rad < 0)
            ang_end_rad += 2 * M_PI;

        double turn_rad = get_turn_rad(ang_start_rad, ang_end_rad, left_turn);

        arc_t out;
        out.pos = pos;
        out.ang_start_rad = -ang_start_rad;
        out.ang_end_rad = -ang_start_rad + turn_rad;
        if(left_turn)
        {
            std::swap(out.ang_end_rad, out.ang_start_rad);
        }

        return out;
    }

    inline line_joint_t get_line_joint(vect2_t pq, vect2_t ps, vect2_t pa, vect2_t pb,
                                       double radius, double str_join_deg = 5)
    {
        line_joint_t out = {};

        out.turn_radius = radius;

        vect2_t qs = ps - pq;
        qs = qs.get_unit();
        vect2_t ab = pb - pa;
        ab = ab.get_unit();
        vect2_t sb = pb - ps;
        sb = sb.get_unit();
        vect2_t sa = pa - ps;
        sa = sa.get_unit();

        double c = qs.cross_prod(sb);
        double c1 = qs.cross_prod(sa);
        bool left_turn = false;

        if(abs(c1) < sin(str_join_deg * DEG_TO_RAD))
        {
            left_turn = c > 0;
        }
        else
        {
            left_turn = c1 > 0;
        }

        if ((sa.x == 0 && sa.y == 0) || (abs(c) < sin(str_join_deg * DEG_TO_RAD) && 
            qs.dot_prod(sb) >= 0 && 
            abs(qs.cross_prod(sa)) < sin(str_join_deg * DEG_TO_RAD)))
        {
            out.tp = JointType::LINE;
            out.line.start = ps;
            out.line.end = pa;
            return out;
        }

        vect2_t r1 = {qs.y, -qs.x}; // unit vector
        if (left_turn)
        {
            r1 = r1.scmul(-1);
        }
        vect2_t r2 = {-ab.y, ab.x}; // unit vector
        if (left_turn)
        {
            r2 = r2.scmul(-1);
        }

        vect2_t po1 = r1.scmul(radius) + ps;
        vect2_t vec_to_ab;
        double dist = get_vec_to_line(po1, pa, pb, &vec_to_ab);

        double r_cp = vec_to_ab.dot_prod(r2);

        double nml_offs = 0;

        if (r_cp > 0)
        {
            nml_offs = dist + radius;
        }
        else
        {
            nml_offs = dist - radius;
        }

        if (nml_offs <= 2 * radius)
        {

            double lat_offs = sqrt(4 * radius * radius - nml_offs * nml_offs);

            vect2_t po2 = po1 + vec_to_ab.scmul(nml_offs) + ab.scmul(lat_offs);

            vect2_t ctr_vec = po2 - po1;
            ctr_vec = ctr_vec.scmul(0.5);

            vect2_t ln_start = po2 + r2.scmul(-radius);

            out.tp = JointType::CIRC_CIRC;
            out.arc1 = get_circ_from_pts(po1, ps, po1 + ctr_vec, left_turn);
            out.arc2 = get_circ_from_pts(po2, po2 + ctr_vec.scmul(-1),
                                         ln_start, !left_turn);

            out.line.start = ln_start;
            out.line.end = pb;
        }
        else
        {
            out.tp = JointType::CIRC;
            out.arc1 = get_circ_from_pts(po1, ps, po1 + r2.scmul(radius), left_turn);

            out.line.start = po1 + r2.scmul(dist);
            out.line.end = pb;
        }

        return out;
    }

    // Projection functions

    inline vect2_t get_projection(double brng_rad, double dist_nm)
    {
        return {dist_nm * sin(brng_rad), dist_nm * cos(brng_rad)};
    }

    inline vect2_t project_point(geo::point tgt, geo::point p_ctr, double brng_add_rad=0.0)
    {
        double brng_rad = p_ctr.get_gc_bearing_rad(tgt)+brng_add_rad;
        double dist_nm = p_ctr.get_gc_dist_nm(tgt);

        return get_projection(brng_rad, dist_nm);
    }
} // namespace geom
