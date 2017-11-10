#include <vector>
#include <cmath>
#include <tuple>

#include "Obstacle.h"
#include "Squircle.h"

bool Squircle::isObstacled(double x, double y)
{
    double ted_x = x - GetX();
    double ted_y = y - GetY();
    double roted_x = ted_x*cos(GetTheta()) + ted_y*sin(GetTheta());
    double roted_y = -ted_x*sin(GetTheta()) + ted_y*cos(GetTheta());
    if( ((2*roted_x/length)*(2*roted_x/length) + (2*roted_y/width)*(2*roted_y/width) + sqrt( pow((2*roted_x/length),4) + pow((2*roted_y/width),4) + (2-4*s)*(2*roted_x/length)*(2*roted_x/length)*(2*roted_y/width)*(2*roted_y/width) ))/2-1 <=0 ) return true;
    else return false;
}

tuple<double,double> Squircle::GetCornerPoint(int index)
{
    if(index==0) return make_tuple(GetX() - (length/2)*cos(GetTheta()) + (width/2)*sin(GetTheta()) , GetY() - (length/2)*sin(GetTheta()) - (width/2)*cos(GetTheta()));
    else if(index==1) return make_tuple(GetX() + (length/2)*cos(GetTheta()) + (width/2)*sin(GetTheta()) , GetY() + (length/2)*sin(GetTheta()) - (width/2)*cos(GetTheta()));
    else if(index==2) return make_tuple(GetX() + (length/2)*cos(GetTheta()) - (width/2)*sin(GetTheta()) , GetY() + (length/2)*sin(GetTheta()) + (width/2)*cos(GetTheta()));
    else return make_tuple(GetX() - (length/2)*cos(GetTheta()) + (width/2)*sin(GetTheta()) , GetY() - (length/2)*sin(GetTheta()) - (width/2)*cos(GetTheta()));
}

double Squircle::GetBeta(double x, double y)
{
    double ted_x = x - GetX();
    double ted_y = y - GetY();
    double roted_x = ted_x*cos(GetTheta()) + ted_y*sin(GetTheta());
    double roted_y = -ted_x*sin(GetTheta()) + ted_y*cos(GetTheta());
    return ((2*roted_x/length)*(2*roted_x/length) + (2*roted_y/width)*(2*roted_y/width) + sqrt( pow((2*roted_x/length),4) + pow((2*roted_y/width),4) + (2-4*s)*(2*roted_x/length)*(2*roted_x/length)*(2*roted_y/width)*(2*roted_y/width) ))/2-1;
}
