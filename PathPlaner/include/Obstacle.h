#ifndef __OBSTACLE
#define __OBSTACLE

#include <vector>

using namespace std;

class Obstacle
{
    double x;
    double y;
    double theta;
public:
    Obstacle():x(0),y(0),theta(0) {}
    Obstacle(double _x, double _y, double _theta):x(_x),y(_y),theta(_theta) {}
    Obstacle(const Obstacle& other) {x=other.x;y=other.y;theta=other.theta;}

    double GetX() const {return x;}
    double GetY() const {return y;}
    double GetTheta() const {return theta;}
    void SetX(double _x) {x=_x;}
    void SetY(double _y) {y=_y;}
    void SetTheta(double _theta) {theta=_theta;}
    
    virtual bool isObstacled(double _x, double _y) = 0;
    virtual double GetBeta(double _x, double _y) = 0;
};

#endif
