#pragma once

#define _USE_MATH_DEFINES
#include <math.h>


namespace geom
{
    struct vect2_t
    {
        double x, y;


        double absval()
        {
            return sqrt(x * x + y * y);
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
} // namespace geom
