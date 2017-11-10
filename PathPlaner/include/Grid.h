#ifndef __GRID
#define __GRID

#include <vector>
#include <tuple>
#include <functional>

using namespace std;

class Grid
{
    double x;
    double y;
    double width;
    double height;

    int nx;
    int ny;

    int hashKey;

public:
    unsigned int GValue;
    double HValue;
    double FValue;

    Grid* ParentGrid;

    Grid() {x=0;y=0;width=1;height=1;GValue=-1;FValue=HValue=0;ParentGrid=NULL;}
    Grid(double _x, double _y, double _w, double _h) {x=_x;y=_y;width=_w;height=_h;GValue=-1;FValue=HValue=0;ParentGrid=NULL;}
    Grid(const Grid &other)
    {
        x=other.x;
        y=other.y;
        width = other.width;
        height = other.height;
        nx = other.nx;
        ny = other.ny;
        hashKey = other.hashKey;
        GValue = other.GValue;
        HValue = other.HValue;
        FValue = other.FValue;
    }

    double GetX() const {return x;}
    double GetY() const {return y;}
    void SetX(double _x) {x=_x;}
    void SetY(double _y) {y=_y;}
    void SetWidth(double _w) {width=_w;}
    void SetHeight(double _h) {height=_h;}

    bool isInside(double _x, double _y);
    vector<tuple<double,double>> GetHeighborPos(bool isDiagonal = true) const;

    int GetIndexX() const {return nx;}
    int GetIndexY() const {return ny;}
    void SetIndex(int x, int y) {nx=x;ny=y;}

    int GetHashKey() const {return hashKey;}
    void SetHashKey(int hk) {hashKey = hk;}

    bool operator==(const Grid &other) const;
};

namespace std
{
    template <>
    struct hash<Grid>
    {
        std::size_t operator()(const Grid& k) const
        {
            return hash<int>()(k.GetHashKey());
        }
    };
}


#endif
