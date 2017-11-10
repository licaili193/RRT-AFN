#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <limits>
#include <typeinfo>

#include "Map.h"

using namespace std;

void NF::checkRadius(double x, double y, double &r)
{
    double d = sqrt( (x-S0.GetX())*(x-S0.GetX())+(y-S0.GetY())*(y-S0.GetY()) );
    if(r> S0.GetRadius()-d ) r = S0.GetRadius()-d;

    for(Sphere &s: S_List)
    {
        d = sqrt( (x-s.GetX())*(x-s.GetX())+(y-s.GetY())*(y-s.GetY()) );
        if(r+s.GetRadius()>d)
        {
            double dm = (r+s.GetRadius()-d)/2;
            r -= dm;
            s.SetRadius(s.GetRadius()-dm);
        }
    }
    
    for(Sphere &s: Unparented_S)
    {
        d = sqrt( (x-s.GetX())*(x-s.GetX())+(y-s.GetY())*(y-s.GetY()) );
        if(r+s.GetRadius()>d)
        {
            double dm = r+s.GetRadius()-d;
            r -= dm;
            //s.SetRadius(s.GetRadius()-dm);
        }
    }
}

void NF::BuildNF()
{
    R0.SetTheta(0);
    R0.SetX(range_x/2);
    R0.SetY(range_y/2);
    R0.SetLength(range_x+margin);
    R0.SetWidth(range_y+margin);
    R0.SetS(0.9999);

    S0.SetX(range_x/2);
    S0.SetY(range_y/2);
    S0.SetRadius((max(range_x,range_y)/2+margin)*1.5);

    for(Obstacle* o: *obsFromWS)
    {
        if(typeid(*o)==typeid(S0)) 
        {
            double sx = o->GetX();
            double sy = o->GetY();
            double rho = ((Sphere*)o)->GetRadius();
            //checkRadius(sx,sy,rho);
            Unparented_S.push_back(Sphere(sx,sy,0.0,rho));
        }
        else
        {
            double sx = o->GetX();
            double sy = o->GetY();
            double rho = min(((Squircle*)o)->GetWidth(),((Squircle*)o)->GetLength())/2.1;
            checkRadius(sx,sy,rho);
            S_List.push_back(Sphere(sx,sy,0.0,rho));
            R_List.push_back(*((Squircle*)o));   
        }
    }

    kappa = (R_List.size()+1)*(range_x*range_y)*fixR*1000;
    skappa = (S_List.size()+Unparented_S.size()+1)*fixS;
}

double NF::Beta_S(double x, double y)
{
    double beta = 1;
    for(int i=0;i<S_List.size();i++) beta*=S_List[i].GetBeta(x,y);
    for(int i=0;i<Unparented_S.size();i++) beta*=Unparented_S[i].GetBeta(x,y);
    beta*=-S0.GetBeta(x,y);
    return beta;
}

double NF::Beta_R(double x, double y)
{
    double beta = 1;
    for(int i=0;i<R_List.size();i++) beta*=R_List[i].GetBeta(x,y);
    beta*=-R0.GetBeta(x,y);
    return beta;
}

double NF::Gamma(double x, double y)
{
    return (x-target_x)*(x-target_x)+(y-target_y)*(y-target_y);
}

double NF::NF_S(double x, double y, double skappa)
{
    double gamma = Gamma(x,y);
    double beta = Beta_S(x,y);
    return gamma/ pow((pow(gamma,skappa)+beta),1/skappa);
}

double NF::NF_R(double x, double y, double kappa, double skappa)
{
    double Fx = 0, Fy = 0;

    double gamma = Gamma(x,y);
    double beta = Beta_R(x,y)*gamma;

    double s = 1;
    double t0,t0x,t0y,bb0,s0;
    if(x==S0.GetX()&&y==S0.GetY())
    {
        t0x = x;
        t0y = y;
    }
    else 
    {
        t0 = S0.GetRadius()*sqrt(1+R0.GetBeta(x,y))/sqrt((x-S0.GetX())*(x-S0.GetX())+(y-S0.GetY())*(y-S0.GetY()));
        t0x = t0*(x-S0.GetX())+S0.GetX();
        t0y = t0*(y-S0.GetY())+S0.GetY();
    }
    bb0 = -beta/R0.GetBeta(x,y);
    s0 = bb0/(bb0-kappa*R0.GetBeta(x,y));
    Fx+=s0*t0x;
    Fy+=s0*t0y;
    s-=s0;

    for(int i=0;i<S_List.size();i++)
    {
        if(x==S_List[i].GetX()&&y==S_List[i].GetY())
        {
            t0x = x;
            t0y = y;
        }
        else 
        {
            t0 = S_List[i].GetRadius()*sqrt(1+R_List[i].GetBeta(x,y))/sqrt((x-S_List[i].GetX())*(x-S_List[i].GetX())+(y-S_List[i].GetY())*(y-S_List[i].GetY()));
            t0x = t0*(x-S_List[i].GetX())+S_List[i].GetX();
            t0y = t0*(y-S_List[i].GetY())+S_List[i].GetY();
        }
        bb0 = beta/R_List[i].GetBeta(x,y);
        s0 = bb0/(bb0+kappa*R_List[i].GetBeta(x,y));
        Fx+=s0*t0x;
        Fy+=s0*t0y;
        s-=s0;
    }
    Fx+=s*x;
    Fy+=s*y;
    return NF_S(Fx,Fy,skappa);
}

double NF::GetPotential(double x, double y)
{
    return NF_R(x,y,kappa,skappa);
}

double NF::GetSpherePotential(double x, double y)
{
    return NF_S(x,y,skappa);
}

double NF::GetSquarePotential(double x, double y)
{
	double gamma = Gamma(x, y);
	double beta = pow(Beta_R(x, y),0.5/(double)(R_List.size()+1));
	return gamma / pow((pow(gamma, skappa) + beta), 1/skappa);
}

tuple<double, double> NF::GetNGradient(double x, double y)
{
	double ep2 = 2 * epsilon;
	double x1 = GetPotential(x + epsilon, y);
	double x2 = GetPotential(x - epsilon, y);
	double dx = (x2 - x1) / ep2;
	double y1 = GetPotential(x, y + epsilon);
	double y2 = GetPotential(x, y - epsilon);
	double dy = (y2 - y1) / ep2;
	double norm = sqrt(dx*dx + dy*dy);
	if (norm == 0) return make_tuple(0.0, 0.0);
	return make_tuple(dx/norm,dy/norm);
}

tuple<double, double> NF::GetNGradientSquare(double x, double y)
{
	double ep2 = 2 * epsilon;
	double x1 = GetSquarePotential(x + epsilon, y);
	double x2 = GetSquarePotential(x - epsilon, y);
	double dx = (x2 - x1) / ep2;
	double y1 = GetSquarePotential(x, y + epsilon);
	double y2 = GetSquarePotential(x, y - epsilon);
	double dy = (y2 - y1) / ep2;
	double norm = sqrt(dx*dx + dy*dy);
	if (norm == 0) return make_tuple(0.0, 0.0);
	return make_tuple(dx / norm, dy / norm);
}
