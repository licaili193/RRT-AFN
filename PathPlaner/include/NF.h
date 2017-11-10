#ifndef __NF
#define __NF

#include <vector>
#include <tuple>

#include "Workspace.h"
#include "Obstacle.h"
#include "Squircle.h"
#include "Sphere.h"

using namespace std;

class NF
{
    vector<Obstacle*>* obsFromWS;
    double range_x;
    double range_y;
    double margin;

    double target_x;
    double target_y;

    double fixS;
    double fixR;
    double epsilon;

    Squircle R0;
    Sphere S0;
    vector<Sphere>S_List;
    vector<Squircle>R_List;
    vector<Sphere>Unparented_S;

    void checkRadius(double x, double y, double &r);
    double NF_S(double x, double y, double kappa);
    double NF_R(double x, double y, double kappa, double skappa);

    double Beta_S(double x, double y);
    double Beta_R(double x, double y);

    double kappa;
    double skappa;
public:
    NF() 
    {
        obsFromWS=NULL;
        range_x=1;
        range_y=1;
        margin=0.1;
        fixS=1;
        fixR=1;
        target_x=0;
        target_y=0;
        epsilon=0.0000001;
    }

    NF(vector<Obstacle*>* a, double x, double y) 
    {
        obsFromWS = a;
        range_x=x;
        range_y=y;
        margin=0.1;
        fixS=1;
        fixR=1;
        target_x=0;
        target_y=0;
        epsilon=0.0000001;
    }

    void ResetNF(vector<Obstacle*>* a, double x, double y) {obsFromWS = a;range_x=x;range_y=y;}

    void SetOriginalObs(vector<Obstacle*>* a) {obsFromWS = a;}
    void SetRange(double x, double y) {range_x=x;range_y=y;}
    void SetMargin(double m) {margin=m;}
    void SetFixParameter(double s, double r) {fixS=s;fixR=r;}
    void SetTarget(double x, double y) {target_x=x;target_y=y;}

    void BuildNF();

    double Gamma(double x, double y);

    double GetPotential(double x, double y);
    double GetSpherePotential(double x, double y); 
	double GetSquarePotential(double x, double y);

	tuple<double, double> GetNGradient(double x, double y);
	tuple<double, double> GetNGradientSquare(double x, double y);
};

#endif