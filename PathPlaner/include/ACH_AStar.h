#ifndef __ACH_AStar
#define __ACH_AStar

#include <utility>
#include <tuple>

#include "Workspace.h"
#include "Grid.h"
#include "NF.h"
#include "Map.h"

using namespace std;

class ACH_AStar
{
    tuple<double,double>sat;
    tuple<int,int>st;
    tuple<int,int>et;

	Map* mp;
    Workspace* ws;
    NF* nf;

    double scale;
public:
	ACH_AStar(Map* m, NF* n, double s) { mp = m; ws = &(m->workspace); nf = n; scale = s; }
	void Reset(Map* m, NF* n, double s) { mp = m; ws = &(m->workspace); nf = n; scale = s; }

    bool StartSearch(double startX, double startY, double endX, double endY, vector<int>&res);
	bool StartSearch2(double startX, double startY, double endX, double endY, vector<int>&res);
	bool StartSearch3(double startX, double startY, double endX, double endY, vector<int>&res);
};

#endif