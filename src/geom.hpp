#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <libnav/geo_utils.hpp>
#include <assert.h>


namespace geom
{
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
            double bi = ih / cos_alpha;
            double r = (ln_dist * bi) / (ln_dist + bi);
            double im = (bi - r) * cos_alpha;

            vect2_t pi = pb + n_b.scmul(bi);
            *out = pi + a.scmul(-im);
            return r;
        }
    }

    // Projection functions

    inline vect2_t get_projection(double brng_rad, double dist_nm)
    {
        return {dist_nm * sin(brng_rad), dist_nm * cos(brng_rad)};
    }

    inline vect2_t project_point(geo::point tgt, geo::point p_ctr)
    {
        double brng_rad = p_ctr.get_gc_bearing_rad(tgt);
        double dist_nm = p_ctr.get_gc_dist_nm(tgt);

        return get_projection(brng_rad, dist_nm);
    }
} // namespace geom
