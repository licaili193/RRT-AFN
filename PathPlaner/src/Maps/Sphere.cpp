#include <vector>
#include <cmath>

#include "Obstacle.h"
#include "Sphere.h"

bool Sphere::isObstacled(double x, double y)
{
    double ted_x = x - GetX();
    double ted_y = y - GetY();
    if(ted_x*ted_x+ted_y*ted_y<=radius*radius ) return true;
    else return false;
}

double Sphere::GetBeta(double x, double y)
{
    double ted_x = x - GetX();
    double ted_y = y - GetY();
    return ted_x*ted_x+ted_y*ted_y-radius*radius;
}