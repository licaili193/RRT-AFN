#ifndef __MAP
#define __MAP

#include <vector>
#include <string>

#include "Workspace.h"
#include "Obstacle.h"
#include "NF.h"

using namespace std;

struct Map
{
    Workspace workspace;
    vector<Obstacle*>obstacles;

    bool isObstacled(double x, double y);

    void GenerateGridMap(vector<vector<char>>&grids);
    bool GenerateGridMap(string path);

    bool GenerateNFMap(string path, NF &nf);
    bool GenerateSNFMap(string path, NF &nf);

    ~Map()
    {
        if(obstacles.empty()) return;
        else
        {
            for(Obstacle* o: obstacles)
            {
                delete o;
            }
        }
    }
};

#endif
