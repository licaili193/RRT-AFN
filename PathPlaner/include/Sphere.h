#ifndef __SPHERE
#define __SPHERE

#include <vector>
#include <tuple>

#include "Obstacle.h"

using namespace std;

class Sphere : public Obstacle
{
    double radius;
public:
    Sphere():Obstacle() {radius=1;}
    Sphere(double _x, double _y, double _theta):Obstacle(_x,_y,_theta) {radius=1;}
    Sphere(double _x, double _y, double _theta, double _radius):Obstacle(_x,_y,_theta) {radius=_radius;}
    Sphere(const Sphere &other):Obstacle(other) {radius=other.radius;}

    double GetRadius() const {return radius;}
    void SetRadius(double _radius) {radius=_radius;}

    bool isObstacled(double x, double y);
    double GetBeta(double x, double y);
};

#endif
