#include <tuple>

#include "Workspace.h"

using namespace std;

tuple<double,double> Workspace::GetGridCenter(int i, int j)
{
    return make_tuple(gs_w*(double)i+gs_w/2,gs_h*(double)j+gs_h/2);
}

tuple<int,int> Workspace::GetGridIndex(double x, double y)
{
    return make_tuple((int)(x/gs_w),(int)(y/gs_h));
}

bool Workspace::isInside(double x, double y) const
{
    if( (x>=0&&x<width) && (y>=0&&y<height) ) return true;
    else return false;
}
    
bool Workspace::isBelong(int i, int j) const
{
    if( (i>=0&&i<dim_x) && (j>=0&&j<dim_y) ) return true;
    else return false;
}

int Workspace::GetGridCombinedIndex(int i, int j) const
{
    if(isBelong(i,j)) return i*dim_x+j;
    else return -1;
}
