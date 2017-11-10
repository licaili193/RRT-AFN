#ifndef __WORKSPACE
#define __WORKSPACE

#include <tuple>

using namespace std;

class Workspace
{
    double width;
    double height;
    int dim_x;
    int dim_y;

    double gs_w;
    double gs_h;

public:
    Workspace() {width=1;height=1;dim_x=1;dim_y=1;gs_w=1;gs_h=1;}
    Workspace(double w, double h, int x, int y) {width=w;height=h;dim_x=x;dim_y=y;gs_w=w/((double)x);gs_h=h/((double)y);}

    void ResetWorkspace(double w, double h, int x, int y) {width=w;height=h;dim_x=x;dim_y=y;gs_w=w/((double)x);gs_h=h/((double)y);}

    tuple<double,double> GetGridCenter(int i, int j);
    tuple<int,int> GetGridIndex(double x, double y);

    bool isInside(double x, double y) const;
    bool isBelong(int i, int j) const;

    int GetGridCombinedIndex(int i, int j) const;

    double GetGridWidth() const {return gs_w;}
    double GetGridHeight() const {return gs_h;}
    double GetWidth() const {return width;}
    double GetHeight() const {return height;}
    int GetDimX() const {return dim_x;}
    int GetDimY() const {return dim_y;}
};

#endif
