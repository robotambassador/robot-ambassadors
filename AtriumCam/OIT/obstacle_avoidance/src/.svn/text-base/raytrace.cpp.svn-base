#include "raytrace.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <cfloat>

#include "ray.hpp"

const int FRAME_SIZE = 256;
const float PI = (float)3.14159265;
const float RANGE = 100000;

void renderScene(std::vector<ObjectPtr> & objects, float sweepAngle,
                 float deltaTheta, float rX, float rY, float rTheta )
{
    float curAngle =  rTheta-(sweepAngle/2.0);
    int numSteps = (int) (sweepAngle/deltaTheta);

    std::cout << "numsteps " << numSteps << std::endl;
    std::vector<float> depths;
    
    std::cout << "intersecting with " << objects.size() << " line(s)\n";
    for(int i=0; i<=numSteps; ++i)
    {
        float rX2 = rX+RANGE*cos(curAngle*(PI/180.0));
        float rY2 = rY+RANGE*sin(curAngle*(PI/180.0));

        RayPtr curRay = RayPtr( new Ray(rX, rY, 0, rX2, rY2, 0) );
        //TODO: make this better
        float closest = FLT_MAX;
        ObjectPtr closestObj;
        //Color curColor = Color();
        float t = FLT_MAX;
        for( std::vector<ObjectPtr>::const_iterator i = objects.begin(); i != objects.end(); ++i )
        {
            t = (*i)->getIntersection(curRay);

            if (t > 0 && t < closest)
            {
                closest = t;
                closestObj = (*i);
            }

        }
        depths.push_back(closest);
        curAngle = curAngle + deltaTheta;
    }

    curAngle =  rTheta-(sweepAngle/2.0);
    for( std::vector<float>::const_iterator i = depths.begin(); i != depths.end(); ++i )
    {
        std::cout << "angle: " << curAngle << " depth " << (*i) << std::endl;
        curAngle = curAngle + deltaTheta;
    }
}
