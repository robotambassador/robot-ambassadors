#ifndef LINE_HPP
#define LINE_HPP

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <cstdlib>
#include "object.hpp"
#include "raytrace.hpp"

class Line : public Object {
private:
    float x1, x2, y1, y2;

public:
    Line(float x1, float x2, float y1, float y2 ) 
        : Object(x1, y1, 0), x1(x1), x2(x2), y1(y1), y2(y2) {}

    float getIntersection(RayPtr r) 
    {
        float xD1,yD1,xD2,yD2,xD3,yD3;  
        float dot,deg,len1,len2;  
        float segmentLen1,segmentLen2;  
        float ua,ub,div;  

        // calculate differences  
        xD1=r->x_1.x()-r->x_0.x();  
        xD2=x2-x1;  
        yD1=r->x_1.y()-r->x_0.y();  
        yD2=y2-y1;  
        xD3=r->x_0.x()-x1;  
        yD3=r->x_0.y()-y1;    

        // calculate the lengths of the two lines  
        len1=sqrt(xD1*xD1+yD1*yD1);  
        len2=sqrt(xD2*xD2+yD2*yD2);  

        // calculate angle between the two lines.  
        dot=(xD1*xD2+yD1*yD2); // dot product  
        deg=dot/(len1*len2);  

        // if abs(angle)==1 then the lines are parallell,  
        // so no intersection is possible  
        if(fabs(deg)==1) return FLT_MAX;  

        // find intersection Pt between two lines  
        div=yD2*xD1-xD2*yD1;  
        ua=(xD2*yD3-yD2*xD3)/div;  
        ub=(xD1*yD3-yD1*xD3)/div;  
        Point pt=Point(r->x_0.x()+ua*xD1,r->x_0.y()+ua*yD1);  

        // calculate the combined length of the two segments  
        // between Pt-p1 and Pt-p2  
        xD1=pt.x()-r->x_0.x();  
        xD2=pt.x()-r->x_1.x();  
        yD1=pt.y()-r->x_0.y();  
        yD2=pt.y()-r->x_1.y();  
        segmentLen1=sqrt(xD1*xD1+yD1*yD1)+sqrt(xD2*xD2+yD2*yD2);  

        // calculate the combined length of the two segments  
        // between Pt-p3 and Pt-p4  
        xD1=pt.x()-x1;  
        xD2=pt.x()-x2;  
        yD1=pt.y()-y1;  
        yD2=pt.y()-y2;  
        segmentLen2=sqrt(xD1*xD1+yD1*yD1)+sqrt(xD2*xD2+yD2*yD2);  

        // if the lengths of both sets of segments are the same as  
        // the lenghts of the two lines the point is actually  
        // on the line segment.  

        // if the point isn’t on the line, return null  
        if(fabs(len1-segmentLen1)>0.01 || fabs(len2-segmentLen2)>0.01)  
            return FLT_MAX;  

        //return distance from intersection pt to robot 
        Point dist = Point(r->x_0.x(), r->x_0.y());
        dist -= pt;
        return magnitude(dist);
    }
};

typedef boost::shared_ptr<Line> LinePtr;

#endif
