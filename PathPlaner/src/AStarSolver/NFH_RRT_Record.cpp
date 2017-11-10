#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <cmath>
#include <tuple>
#include <ctime>
#include <limits>
#include <iostream>

#include "NFH_RRT.h"
#include "kdtree.h"

using namespace std;
using namespace lemon;

string Double2String(double a)
{
	ostringstream strs;
	strs << a;
	return strs.str();
}

double DistQR(double a, double b, double c, double d)
{
	return pow((a - c),2) + pow((b - d),2);
}

void WriteLine(double x1, double y1, double x2, double y2, double midx, double midy, double ox1, double oy1, ofstream &file)
{
	file << Double2String(x1) << "\t";
	file << Double2String(y1) << "\t";
	file << Double2String(x2) << "\t";
	file << Double2String(y2) << "\t";
	file << Double2String(midx) << "\t";
	file << Double2String(midy) << "\t";
	file << Double2String(ox1) << "\t";
	file << Double2String(oy1) << "\t";
	file << "\n";
}

bool NFH_RRT::traceRecord(double midx, double midy, size_t index1, size_t index2, ofstream &file)
{
	if (index1 > index || index2 > index) return false;
	double x1 = pos_list[index1 * 2];
	double y1 = pos_list[index1 * 2 + 1];
	double x2 = pos_list[index2 * 2];
	double y2 = pos_list[index2 * 2 + 1];
	nf->SetTarget(midx, midy);
	size_t temp_index, temp_index2;
	ListDigraph::NodeMap<size_t>&sm = startMap;
	ListDigraph::NodeMap<size_t>&em = endMap;

	double oir_x1 = x1;
	double oir_y1 = y1;
	double oir_x2 = x2;
	double oir_y2 = y2;

	double oldx = numeric_limits<double>::max();
	double oldy = numeric_limits<double>::max();
	double o2x, o2y;
	WriteLine(oir_x1, oir_y1, numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), midx, midy, oir_x1, oir_y1, file);/////////////////////
	bool isn1 = true;
	while (!isNear(midx, midy, x1, y1, step_len))
	{
		tuple<double, double>dir = nf->GetNGradientSquare(x1, y1);/////////////////////Test square gradient
		//tuple<double, double>dir = nf->GetNGradient(x2, y2);/////////////////////Test square gradient
		double t1 = x1 + step_len*get<0>(dir);
		double t2 = y1 + step_len*get<1>(dir);
		double se_pt[] = { t1,t2 };
		struct kdres *presults = kd_nearest(pstartTree, se_pt);
		double pos1[2];
		size_t *pch1 = (size_t*)kd_res_item(presults, pos1);
		kd_res_free(presults);
		size_t temp = *pch1;
		oldx = pos_list[temp * 2];
		oldy = pos_list[temp * 2 + 1];

		if (isNear(t1, t2, oldx, oldy, step_len / 2))
		{
			isn1 = false;
			break;
		}
		x1 = t1;
		y1 = t2;
		if (!ws->isInside(x1, y1) || mp->isObstacled(x1, y1))
		{
			isn1 = false;
			break;
		}
		if (index >= max_size) return false;
		WriteLine(x1, y1, numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), midx, midy, oir_x1, oir_y1, file);/////////////////////
		pos_list[index * 2] = x1;
		pos_list[index * 2 + 1] = y1;
		index_list[index] = index;
		ListDigraph::Node u = startGraph.addNode();
		ListDigraph::Arc a = startGraph.addArc(node_list[node1], u);
		startArcMap[a] = 1;
		node_list[index] = u;
		node1 = index;
		sm[u] = index;
		kd_insert(pstartTree, &pos_list[2 * index], &index_list[index]);
		index++;
	}
	if (isn1)
	{
		if (index >= max_size) return false;
		x1 = midx;
		y1 = midy;
		pos_list[index * 2] = midx;
		pos_list[index * 2 + 1] = midy;
		index_list[index] = index;
		WriteLine(midx, midy, numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), midx, midy, oir_x1, oir_y1, file);//////////
		ListDigraph::Node u = startGraph.addNode();
		ListDigraph::Arc a = startGraph.addArc(node_list[node1], u);
		startArcMap[a] = 1;
		node_list[index] = u;
		node1 = index;
		sm[u] = index;
		kd_insert(pstartTree, &pos_list[2 * index], &index_list[index]);
		temp_index = index;
		index++;
	}
	//bool changeFlag = false;
	//size_t changeIndex;
	//{
	//	double se_pt[] = { x2,y2 };
	//	struct kdres *presults = kd_nearest(pstartTree, se_pt);
	//	double pos1[2];
	//	size_t *pch1 = (size_t*)kd_res_item(presults, pos1);
	//	kd_res_free(presults);
	//	size_t temp = *pch1;
	//	changeIndex = temp;
	//	if(DistQR(oir_x2,oir_y2, pos_list[temp * 2], pos_list[temp * 2 + 1])<DistQR(oir_x2, oir_y2, midx, midy))
	//	{
	//		midx = pos_list[temp * 2];
	//		midy = pos_list[temp * 2 + 1];
	//		nf->SetTarget(midx, midy);
	//		changeFlag = true;
	//	}
	//}
	WriteLine(numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), oir_x2, oir_y2, midx, midy, oir_x2, oir_y2, file);/////////////////////
	bool isn2 = true;
	while (!isNear(midx, midy, x2, y2, step_len))
	{
		tuple<double, double>dir = nf->GetNGradientSquare(x2, y2);/////////////////////Test square gradient
		//tuple<double, double>dir = nf->GetNGradient(x2, y2);/////////////////////Test square gradient
		double t1 = x2 + step_len*get<0>(dir);
		double t2 = y2 + step_len*get<1>(dir);
		double se_pt[] = { t1,t2 };
		struct kdres *presults = kd_nearest(pendTree, se_pt);
		double pos1[2];
		size_t *pch1 = (size_t*)kd_res_item(presults, pos1);
		kd_res_free(presults);
		size_t temp = *pch1;
		oldx = pos_list[temp * 2];
		oldy = pos_list[temp * 2 + 1];
		if (isNear(t1, t2, oldx, oldy, step_len/2))
		{
			isn2 = false;
			break;
		}
		x2 = t1;
		y2 = t2;
		if (!ws->isInside(x2, y2) || mp->isObstacled(x2, y2))
		{
			isn2 = false;
			break;
		}
		if (index >= max_size) return false;
		WriteLine(numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), x2, y2, midx, midy, oir_x2, oir_y2, file);//////////
		pos_list[index * 2] = x2;
		pos_list[index * 2 + 1] = y2;
		index_list[index] = index;
		ListDigraph::Node u = endGraph.addNode();
		ListDigraph::Arc a = endGraph.addArc(u, node_list[node2]);
		endArcMap[a] = 1;
		node_list[index] = u;
		node2 = index;
		em[u] = index;
		kd_insert(pendTree, &pos_list[2*index], &index_list[index]);
		index++;
	}
	if (isn2)
	{
		if (index >= max_size) return false;
		pos_list[index * 2] = midx;
		pos_list[index * 2 + 1] = midy;
		index_list[index] = index;
		WriteLine(numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), midx, midy, midx, midy, oir_x2, oir_y2, file);//////////
		ListDigraph::Node u = endGraph.addNode();
		ListDigraph::Arc a = endGraph.addArc(u, node_list[node2]);
		endArcMap[a] = 1;
		node_list[index] = u;
		node2 = index;
		em[u] = index;
		kd_insert(pendTree, &pos_list[2*index], &index_list[index]);
		temp_index2 = index;
		index++;
	}

	if (isn1&&isn2) bridges.push_back(make_pair(temp_index, temp_index2));
	//if (isn1&&isn2)
	//{
	//	if (!changeFlag) bridges.push_back(make_pair(temp_index, temp_index2));
	//	else bridges.push_back(make_pair(changeIndex, temp_index2));
	//}
	return true;
}

bool NFH_RRT::StartRecord(double startX, double startY, double endX, double endY, string Path, bool isStopWhenFound)
{
	if ((!ws->isInside(startX, startY)) || (!ws->isInside(endX, endY))) return false;
	if (mp->isObstacled(startX, startY) || mp->isObstacled(endX, endY)) return false;

	ofstream file;
	file.open(Path, ios::out | ios::trunc);
	if (!file.is_open()) return false;

	srand(time(NULL));
	iter_index = 0;

	index = 0;

	pos_list = new double[max_size * 2]();
	index_list = new size_t[max_size];
	node_list = new ListDigraph::Node[max_size];
	index_list[0] = 0;
	index_list[1] = 1;
	pos_list[0] = startX;
	pos_list[1] = startY;
	pos_list[2] = endX;
	pos_list[3] = endY;
	index = 2;

	startGraph.clear();
	endGraph.clear();

	pstartTree = kd_create(2);
	pendTree = kd_create(2);

	bridges.clear();
	ListDigraph::Node u = startGraph.addNode();
	node_list[0] = u;
	startMap[u] = 0;
	kd_insert(pstartTree, &pos_list[0], &index_list[0]);
	ListDigraph::Node v = endGraph.addNode();
	node_list[1] = v;
	endMap[v] = 1;
	kd_insert(pendTree, &pos_list[2], &index_list[1]);

	while (true)
	{
		iter_index++;

		double midx = fRand(0, ws->GetWidth());
		double midy = fRand(0, ws->GetWidth());
		if (!ws->isInside(midx, midy) || mp->isObstacled(midx, midy)) continue;
		double se_pt[] = { midx,midy };
		struct kdres *presults = kd_nearest(pstartTree, se_pt);
		double pos1[2];
		size_t *pch1 = (size_t*)kd_res_item(presults, pos1);
		kd_res_free(presults);
		presults = kd_nearest(pendTree, se_pt);
		node1 = *pch1;
		double pos2[2];
		size_t *pch2 = (size_t*)kd_res_item(presults, pos2);
		kd_res_free(presults);
		node2 = *pch2;

		if (!traceRecord(midx, midy, node1, node2, file)) break;
		if (isStopWhenFound && !bridges.empty()) break;
	}
	file.close();

	if (bridges.empty()) return false;
	else return true;
}