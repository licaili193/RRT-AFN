#include <tuple>
#include <vector>

#include "Grid.h"

using namespace std;

bool Grid::isInside(double _x, double _y)
{
    if( (_x>=x-width/2 && _x<x+width/2) && (_y>=y-height/2 && _y<y+height/2) ) return true;
    else return false;
}

vector<tuple<double,double>> Grid::GetHeighborPos(bool isDiagonal) const
{
    if(!isDiagonal)
    {
        vector<tuple<double,double>>res(4);
        res[0] = make_tuple(x-width,y);
        res[1] = make_tuple(x+width,y);
        res[2] = make_tuple(x,y-height);
        res[3] = make_tuple(x,y+height);
        return res;
    }
    else
    {
        vector<tuple<double,double>>res(8);
        res[0] = make_tuple(x-width,y);
        res[1] = make_tuple(x+width,y);
        res[2] = make_tuple(x,y-height);
        res[3] = make_tuple(x,y+height);
        res[4] = make_tuple(x-width,y-height);
        res[5] = make_tuple(x+width,y-height);
        res[6] = make_tuple(x-width,y+height);
        res[7] = make_tuple(x+width,y+height);
        return res;
    }   
}

bool Grid::operator==(const Grid &other) const
{
    return hashKey==other.hashKey;
}