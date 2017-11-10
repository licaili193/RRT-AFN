#ifndef __SQUIRCLE
#define __SQUIRCLE

#include <vector>
#include <tuple>

#include "Obstacle.h"

using namespace std;

class Squircle : public Obstacle
{
    double width;
    double length;
    double s;
public:
    Squircle():Obstacle() {width=0;length=0;s=0.9999;}
    Squircle(double _x, double _y, double _theta):Obstacle(_x,_y,_theta) {width=1;length=1;s=0.9999;}
    Squircle(double _x, double _y, double _theta, double _w, double _h, double _s):Obstacle(_x,_y,_theta) {width=_w;length=_h;s=_s;}
    Squircle(const Squircle &other):Obstacle(other) {width=other.width;length=other.length;s=other.s;}

    double GetWidth() const {return width;}
    double GetLength() const {return length;}
    double GetS() const {return s;}
    void SetWidth(double _w) {width=_w;}
    void SetLength(double _h) {length=_h;}
    void SetS(double _s) {s=_s;}

    bool isObstacled(double x, double y);
    tuple<double,double> GetCornerPoint(int index);
    double GetBeta(double x, double y);
};

#endif
