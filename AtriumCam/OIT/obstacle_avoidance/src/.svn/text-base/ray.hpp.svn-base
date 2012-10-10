#ifndef RAY_HPP
#define RAY_HPP

#include <boost/shared_ptr.hpp>
#include "point.hpp"
#include <iostream>
#include <limits>

//x(t) = x_0 + t(x_1-x_0)

class Ray {
public:
    Point x_0; //eye pt.
    Point x_1;
    bool hitCap;
    int capHit;

public:
    Ray ( float x0, float y0, float z0, float x1, float y1, float z1 ) :
        x_0(Point(x0, y0, z0)), x_1(Point(x1, y1, z1)) {}
        Point e() { return x_0; }
        Point d() 
        { 
            Point temp = x_1;
            temp -= x_0;
            hitCap = false;
            capHit = 0;
            return temp;
        }
    Point evalAtT( float t )
    {
        Point temp = x_1;
        temp = Point(temp.x()*t, temp.y()*t, temp.z()*t);
        //std::cout << "x_1: " << x_1 << " \tx_0 " << x_0 << " \ttemp " << temp << '\n';
        return temp;

    }
};

typedef boost::shared_ptr<Ray> RayPtr;

#endif

