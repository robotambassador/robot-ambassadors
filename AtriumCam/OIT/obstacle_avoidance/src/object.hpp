#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <boost/shared_ptr.hpp>
#include <utility>
#include <iostream>
#include "ray.hpp"

class Object;

typedef boost::shared_ptr<Object> ObjectPtr;

class Object {
protected:
    float x, y, z;
    Point center;
public:
    Object(float xp, float yp, float zp) 
        : x(xp), y(yp), z(zp), center(Point( xp, yp, zp)) 
    {
    }
    virtual ~Object() {}
    virtual float getIntersection(RayPtr r) = 0;
};


#endif

