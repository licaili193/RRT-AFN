#include <map>
#include <set>
#include <utility>
#include <tuple>
#include <limits>
#include <stack>
#include <iostream>
#include <unordered_set>
#include <unordered_map>

#include "Workspace.h"
#include "Grid.h"
#include "NF.h"
#include "ACH_AStar.h"

using namespace std;

bool ACH_AStar::StartSearch(double startX, double startY, double endX, double endY, vector<int>&res)
{
    if((!ws->isInside(startX,startY))||(!ws->isInside(endX,endY))) return false;
    sat = make_tuple(startX,startY);
    st = ws->GetGridIndex(startX,startY);
    et = ws->GetGridIndex(endX,endY);
	nf->SetTarget(endX, endY);

	vector<double>GValue;
	vector<double>FValue;
	vector<int>parents;
	multimap<double, int>openSet;
	unordered_set<int>openTable;
	unordered_set<int>closeSet;

	parents.resize(ws->GetDimX()*ws->GetDimY(), 0);
    FValue.resize(ws->GetDimX()*ws->GetDimY(), 1);
    GValue.resize(ws->GetDimX()*ws->GetDimY(), numeric_limits<double>::max());

    int g_index = ws->GetGridCombinedIndex(get<0>(st),get<1>(st));
	int e_index = ws->GetGridCombinedIndex(get<0>(et), get<1>(et));
    GValue[g_index] = 0;
	FValue[g_index] = 0;
    openSet.insert(make_pair(0,g_index));
	openTable.insert(g_index);

    while(!openSet.empty())
    {
        int cur_index = openSet.begin()->second;
		int di = cur_index / ws->GetDimX();
		int dj = cur_index % ws->GetDimX();
		//cout << "Searching index: " << cur_index << endl;

        if(e_index==cur_index)
        {
			res.clear();
			int index = cur_index;
			while (index != g_index) 
			{
				res.push_back(index); 
				index = parents[index];
			}
			res.push_back(index);
			cout << "Path length: " << res.size() << endl;
			cout << "Close set size: " << closeSet.size() << endl;
            return true;
        }

		openTable.erase(openSet.begin()->second);
        openSet.erase(openSet.begin());
        closeSet.insert(cur_index);

		tuple<int, int> neighbors[8] = { make_tuple(di-1,dj), make_tuple(di+1,dj), make_tuple(di,dj-1),make_tuple(di,dj+1),make_tuple(di-1,dj-1),make_tuple(di+1,dj-1),make_tuple(di-1,dj+1),make_tuple(di+1,dj+1) };
        for(tuple<int, int> pos: neighbors)
        {
			tuple<double,double>cord = ws->GetGridCenter(get<0>(pos), get<1>(pos));
            if(ws->isBelong(get<0>(pos),get<1>(pos)) && !mp->isObstacled(get<0>(cord), get<1>(cord)))
            {
                int comb_int = ws->GetGridCombinedIndex(get<0>(pos),get<1>(pos));
                if(closeSet.find(comb_int)!=closeSet.end()) continue;
				if (openTable.find(comb_int) == openTable.end())
				{
					FValue[comb_int] = nf->GetPotential(get<0>(pos), get<1>(pos));
					openTable.insert(comb_int);
					openSet.insert(make_pair(FValue[comb_int], comb_int));
				}
				int tGValue = GValue[cur_index] + ((cur_index%ws->GetDimX() != get<1>(pos)) && (cur_index / ws->GetDimX() != get<0>(pos))) ? 1.4 : 1.0;
				if (tGValue >= GValue[comb_int]) continue;
				parents[comb_int] = cur_index;
				GValue[comb_int] = tGValue;
            }
        }
    }
	return false;
}

bool ACH_AStar::StartSearch2(double startX, double startY, double endX, double endY, vector<int>&res)
{
	if ((!ws->isInside(startX, startY)) || (!ws->isInside(endX, endY))) return false;
	sat = make_tuple(startX, startY);
	st = ws->GetGridIndex(startX, startY);
	et = ws->GetGridIndex(endX, endY);
	nf->SetTarget(endX, endY);

	//vector<double>GValue;
	vector<int>parents;
	stack<int>openSet;
	unordered_set<int>openTable;
	unordered_set<int>closeSet;

	parents.resize(ws->GetDimX()*ws->GetDimY(), 0);
	//GValue.resize(ws->GetDimX()*ws->GetDimY(), numeric_limits<double>::max());

	int g_index = ws->GetGridCombinedIndex(get<0>(st), get<1>(st));
	int e_index = ws->GetGridCombinedIndex(get<0>(et), get<1>(et));
	//GValue[g_index] = 0;
	openSet.push(g_index);
	openTable.insert(g_index);

	while (!openSet.empty())
	{
		int cur_index = openSet.top();
		int di = cur_index / ws->GetDimX();
		int dj = cur_index % ws->GetDimX();
		//cout << "Searching index: " << cur_index << endl;

		if (e_index == cur_index)
		{
			res.clear();
			int index = cur_index;
			while (index != g_index)
			{
				res.push_back(index);
				index = parents[index];
			}
			res.push_back(index);
			cout << "Path length: " << res.size() << endl;
			cout << "Close set size: " << closeSet.size() << endl;
			return true;
		}

		openSet.pop();
		openTable.erase(cur_index);
		closeSet.insert(cur_index);

		tuple<int, int> neighbors[8] = { make_tuple(di - 1,dj), make_tuple(di + 1,dj), make_tuple(di,dj - 1),make_tuple(di,dj + 1),make_tuple(di - 1,dj - 1),make_tuple(di + 1,dj - 1),make_tuple(di - 1,dj + 1),make_tuple(di + 1,dj + 1) };
		multimap<double, int, std::greater<double>> tempMap;
		for (tuple<int, int> pos : neighbors)
		{
			tuple<double, double>cord = ws->GetGridCenter(get<0>(pos), get<1>(pos));
			if (ws->isBelong(get<0>(pos), get<1>(pos)) && !mp->isObstacled(get<0>(cord), get<1>(cord)))
			{
				int comb_int = ws->GetGridCombinedIndex(get<0>(pos), get<1>(pos));
				if (closeSet.find(comb_int) != closeSet.end()) continue;
				if (openTable.find(comb_int) == openTable.end())
				{
					tempMap.insert(make_pair(nf->GetPotential(get<0>(pos), get<1>(pos)),comb_int));
				}
				//int tGValue = GValue[cur_index] + ((cur_index%ws->GetDimX() != get<1>(pos)) && (cur_index / ws->GetDimX() != get<0>(pos))) ? 1.4 : 1.0;
				//if (tGValue >= GValue[comb_int]) continue;
				parents[comb_int] = cur_index;
				//GValue[comb_int] = tGValue;
			}
		}
		multimap<double, int>::iterator it = tempMap.begin();
		while (it != tempMap.end())
		{
			openSet.push(it->second);
			openTable.insert(it->second);
			it++;
		}
	}
	return false;
}

bool ACH_AStar::StartSearch3(double startX, double startY, double endX, double endY, vector<int>&res)
{
	if ((!ws->isInside(startX, startY)) || (!ws->isInside(endX, endY))) return false;
	sat = make_tuple(startX, startY);
	st = ws->GetGridIndex(startX, startY);
	et = ws->GetGridIndex(endX, endY);
	nf->SetTarget(endX, endY);

	vector<double>GValue;
	vector<double>FValue;
	vector<int>parents;
	unordered_set<int>openSet;
	unordered_set<int>closeSet;

	parents.resize(ws->GetDimX()*ws->GetDimY(), 0);
	FValue.resize(ws->GetDimX()*ws->GetDimY(), numeric_limits<double>::max());
	GValue.resize(ws->GetDimX()*ws->GetDimY(), numeric_limits<double>::max());

	int g_index = ws->GetGridCombinedIndex(get<0>(st), get<1>(st));
	int e_index = ws->GetGridCombinedIndex(get<0>(et), get<1>(et));
	GValue[g_index] = 0;
	FValue[g_index] = 0;
	openSet.insert(g_index);

	while (!openSet.empty())
	{
		double minF = numeric_limits<double>::max();
		int cur_index = 0;
		for ( int p : openSet)
		{
			if (FValue[p] < minF)
			{
				cur_index = p;
				minF = FValue[p];
			}
		}
		int di = cur_index / ws->GetDimX();
		int dj = cur_index % ws->GetDimX();
		//cout << "Searching index: " << cur_index << endl;

		if (e_index == cur_index)
		{
			res.clear();
			int index = cur_index;
			while (index != g_index)
			{
				res.push_back(index);
				index = parents[index];
			}
			res.push_back(index);
			cout << "Path length: " << res.size() << endl;
			cout << "Close set size: " << closeSet.size() << endl;
			return true;
		}

		openSet.erase(cur_index);
		closeSet.insert(cur_index);

		tuple<int, int> neighbors[8] = { make_tuple(di - 1,dj), make_tuple(di + 1,dj), make_tuple(di,dj - 1),make_tuple(di,dj + 1),make_tuple(di - 1,dj - 1),make_tuple(di + 1,dj - 1),make_tuple(di - 1,dj + 1),make_tuple(di + 1,dj + 1) };
		for (tuple<int, int> pos : neighbors)
		{
			tuple<double, double>cord = ws->GetGridCenter(get<0>(pos), get<1>(pos));
			if (ws->isBelong(get<0>(pos), get<1>(pos)) && !mp->isObstacled(get<0>(cord), get<1>(cord)))
			{
				int comb_int = ws->GetGridCombinedIndex(get<0>(pos), get<1>(pos));
				if (closeSet.find(comb_int) != closeSet.end()) continue;
				if (openSet.find(comb_int) == openSet.end()) openSet.insert(comb_int);
				int tGValue = GValue[cur_index] + ((cur_index%ws->GetDimX() != get<1>(pos)) && (cur_index / ws->GetDimX() != get<0>(pos))) ? 1.4 : 1.0;
				if (tGValue >= GValue[comb_int]) continue;
				parents[comb_int] = cur_index;
				GValue[comb_int] = tGValue;
				FValue[comb_int] = tGValue + abs(get<0>(pos) - get<0>(et)) + abs(get<1>(pos) - get<1>(et));
			}
		}
	}
	return false;
}