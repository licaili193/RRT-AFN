#ifndef __NFH_RRT
#define __NFH_RRT

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "Workspace.h"
#include "NF.h"
#include "Map.h"
#include "NFH_RRT.h"
#include "kdtree.h"

using namespace std;
using namespace lemon;

class NFH_RRT
{
	Map* mp;
	Workspace* ws;
	NF* nf;

	kdtree* pstartTree;
	kdtree* pendTree;

	ListDigraph::Node *node_list;
	size_t index;

	size_t node1;
	size_t node2;

	double fRand(double fMin, double fMax);
	bool isNear(double cx, double cy, double tx, double ty, double r);

	bool trace(double midx, double midy, size_t index1, size_t index2);
	bool traceRecord(double midx, double midy, size_t index1, size_t index2, ofstream &file);
public:
	double *pos_list;
	size_t *index_list;
	double step_len;
	size_t max_size;
	ListDigraph startGraph;
	ListDigraph endGraph;
	vector<pair<size_t,size_t>>bridges;
	int iter_index;
	ListDigraph::NodeMap<size_t> startMap;
	ListDigraph::NodeMap<size_t> endMap;
	ListDigraph::ArcMap<int> startArcMap;
	ListDigraph::ArcMap<int> endArcMap;

	NFH_RRT(Map* m, NF* n, double step, size_t size):
		startGraph(), endGraph(), 
		startMap(startGraph), endMap(endGraph), startArcMap(startGraph), endArcMap(endGraph) 
	{
		mp = m; 
		ws = &(m->workspace); 
		nf = n; 
		step_len = step; 
		max_size = size+2; 
		pos_list = NULL;
		index_list = NULL;
		node_list = NULL;
		pstartTree = NULL;
		pendTree = NULL;
	}

	void Reset(Map* m, NF* n, double step, size_t size) { mp = m; ws = &(m->workspace); nf = n; step_len = step; max_size = size+2; }
	void Clear() 
	{
		if (pstartTree)
		{
			kd_free(pstartTree);
			pstartTree = NULL;
		}
		if (pendTree)
		{
			kd_free(pendTree);
			pendTree = NULL;
		}
		if (pos_list) 
		{
			delete[] pos_list; 
			pos_list = NULL;
		}
		if (node_list)
		{
			delete[] node_list;
			node_list = NULL;
		}
		if (index_list)
		{
			delete[] index_list;
			index_list = NULL;
		}
	}
	~NFH_RRT() { Clear(); }

	bool Start(double startX, double startY, double endX, double endY, bool isStopWhenFound = true);
	bool StartRecord(double startX, double startY, double endX, double endY, string Path, bool isStopWhenFound = true);
	bool GetPath(int num, vector<pair<double,double>>&res, double &len);
	bool GetPath(vector<pair<double, double>>&res, double &len);

	size_t GetIndex() const { return index; }
};

double DistQR(double a, double b, double c, double d);

#endif