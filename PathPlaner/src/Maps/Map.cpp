#include <vector>
#include <tuple>
#include <fstream>
#include <sstream>
#include <string>

#include "Map.h"

using namespace std;

bool Map::isObstacled(double x, double y)
{
    if(obstacles.empty()) return false;
    for(auto o: obstacles)
    {
        if(o->isObstacled(x,y)) return true;
    }
    return false;
}

void Map::GenerateGridMap(vector<vector<char>>&grids)
{
    grids.clear();
    grids.resize(workspace.GetDimY(),vector<char>(workspace.GetDimX(),0));
    for(int j=0;j<workspace.GetDimY();j++)
    {
        for(int i=0;i<workspace.GetDimX();i++)
        {
            tuple<double,double> t = workspace.GetGridCenter(i,j);
            if(isObstacled(get<0>(t),get<1>(t))) grids[j][i] = 1;
        }
    }
}

bool Map::GenerateGridMap(string path)
{
    ofstream file;
    file.open (path, ios::out|ios::trunc);
    if(file.is_open())
    {
        for(int j=0;j<workspace.GetDimY();j++)
        {
            for(int i=0;i<workspace.GetDimX();i++)
            {
                tuple<double,double> t = workspace.GetGridCenter(i,j);
                if(isObstacled(get<0>(t),get<1>(t))) file<<"1 ";
                else file<<"0 ";
            }
            file<<"\n";
        }
        file.close();
        return true;
    }
    else return false;
}

bool Map::GenerateNFMap(string path, NF &nf)
{
    ofstream file;
    file.open (path, ios::out|ios::trunc);
    if(file.is_open())
    {
        for(int j=0;j<workspace.GetDimY();j++)
        {
            for(int i=0;i<workspace.GetDimX();i++)
            {
                tuple<double,double> t = workspace.GetGridCenter(i,j);
                if(!isObstacled(get<0>(t),get<1>(t))) 
                {
                    ostringstream strs;
                    strs<<nf.GetPotential(get<0>(t),get<1>(t));
                    string str = strs.str();
                    file<<str<<"\t";
                }
                else file<<"1\t";
            }
            file<<"\n";
        }
        file.close();
        return true;
    }
    else return false;
}

bool Map::GenerateSNFMap(string path, NF &nf)
{
    ofstream file;
    file.open (path, ios::out|ios::trunc);
    if(file.is_open())
    {
        for(int j=0;j<workspace.GetDimY();j++)
        {
            for(int i=0;i<workspace.GetDimX();i++)
            {
                tuple<double,double> t = workspace.GetGridCenter(i,j);
                if(!isObstacled(get<0>(t),get<1>(t))) 
                {
                    ostringstream strs;
                    strs<<nf.GetSpherePotential(get<0>(t),get<1>(t));
                    string str = strs.str();
                    file<<str<<"\t";
                }
                else file<<"1\t";
            }
            file<<"\n";
        }
        file.close();
        return true;
    }
    else return false;
}