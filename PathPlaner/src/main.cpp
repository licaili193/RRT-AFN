#include <iostream>
#include <vector>
#include <ctime>

#include <lemon/list_graph.h>
#include "kdtree.h"

#include "Map.h"
#include "Obstacle.h"
#include "Squircle.h"
#include "Sphere.h"
#include "NF.h"
#include "NFH_RRT.h"

using namespace std;
using namespace lemon;

int main()
{
    cout<<"Welcome"<<endl;
    Map m;
    m.workspace.ResetWorkspace(8.0,8.0,500,500);
	Squircle* s1 = new Squircle(2, 4, 0.0, 5, 1, 0.9999);
	Squircle* s2 = new Squircle(6, 4, 0.0, 5, 1, 0.9999);
	Squircle* s3 = new Squircle(4, 6, 0.0, 1, 5, 0.9999);
	//Squircle* s4 = new Squircle(6.5, 4.7, 0.0, 4.6, 0.6, 0.9999);
	//Squircle* s5 = new Squircle(4, 6.5, 0.0, 0.6, 5.2, 0.9999);
	//Squircle* s6 = new Squircle(3, 4.5, 0.0, 0.6, 3.4, 0.9999);
	m.obstacles.push_back(s1);
	m.obstacles.push_back(s2);
	m.obstacles.push_back(s3);
	//m.obstacles.push_back(s4);
	//m.obstacles.push_back(s5);
	//m.obstacles.push_back(s6);
    //m.GenerateGridMap("test.dat");

    NF nf(&m.obstacles,8.0,8.0);
    nf.SetTarget(7.5,4);
    nf.SetFixParameter(0.1, 1000);
	nf.BuildNF();

	//ACH_AStar as(&m,&nf,1);
	clock_t begin;
	clock_t end;

    //m.GenerateNFMap("testNF.dat",nf);
    //m.GenerateSNFMap("testSNF.dat",nf);
	NFH_RRT nr(&m,&nf,0.1,10000);
	vector<pair<double, double>>res;
	
	vector<double>tm;
	vector<double>nd;
	vector<double>ln;
	int su = 0;
	int tim = 1000;
	for (int i = 0; i < tim; i++)
	{
		begin = clock();
		bool r = nr.Start(4, 4, 4, 7);
		double len;
		if (r) nr.GetPath(0, res, len);
		end = clock();
		if (r)
		{
			tm.push_back(end - begin);
			ln.push_back(len);
			nd.push_back(nr.GetIndex());
			su++;
		}
		nr.Clear();
	}
	double s_tm = 0;
	for (double d : tm) { s_tm += d; }
	double s_nd = 0;
	for (double d : nd) { s_nd += d; }
	double s_ln = 0;
	for (double d : ln) { s_ln += d; }
	cout << tim << " tests: " << tim << " succeded; rate: " << (double)su / (double)tim << endl;
	cout << "Avg. planning time: " << s_tm / (double)tm.size() << endl;
	cout << "Avg. path length: " << s_ln / (double)ln.size() << endl;
	cout << "Avg. node size: " << s_nd / (double)nd.size() << endl;
	
	/*
	cout << "Start..." << endl;
	begin = clock();
	bool r = nr.Start(4, 4, 4, 7);
	double len;
	if (r) nr.GetPath(0, res, len);
	end = clock();
	cout << "Search time: " << (end - begin) << " ms" << endl;
	if (r)
	{
		cout << "Path found with " << nr.iter_index << " iterations" << endl;
		cout << "Path length: " << len << endl;
		cout << "Nodes: " << nr.GetIndex() - 1 << endl;
		//cout << "Path: " << endl;
		//for (pair<double, double>p : res)
		//{
		//	cout << p.first << "\t" << p.second << endl;
		//}
	}
	else cout << "Search failed" << endl;
	nr.Clear();
	*/

	/*
	cout << "Start optimal..." << endl;
	begin = clock();
	bool r = nr.Start(4, 4, 4, 7, false);
	double len;
	if (r) nr.GetPath(res, len);
	end = clock();
	cout << "Search time: " << (end - begin) << " ms" << endl;
	if (r)
	{
		cout << "Path found with " << nr.iter_index << " iterations" << endl;
		cout << "Shortest path length: " << len << endl;
		cout << "Nodes: " << nr.GetIndex() - 1 << endl;
		//cout << "Path: " << endl;
		//for (pair<double, double>p : res)
		//{
		//	cout << p.first << "\t" << p.second << endl;
		//}
	}
	else cout << "Search failed" << endl;
	nr.Clear();
	*/
	/*
	cout << "Start..." << endl;
	//begin = clock();
	bool r = nr.StartRecord(4, 4, 4, 7, "treedata.dat");
	double len;
	if (r) nr.GetPath(0, res, len);
	//end = clock();
	//cout << "Search time: " << (end - begin) << " ms" << endl;
	if (r)
	{
		cout << "Path found with " << nr.iter_index << " iterations" << endl;
		cout << "Path length: " << len << endl;
		cout << "Nodes: " << nr.GetIndex() - 1 << endl;
		ofstream file;
		file.open("pathdata.dat", ios::out | ios::trunc);
		//if (!file.is_open()) return false;
		//cout << "Path: " << endl;
		for (int i = res.size() - 1; i >= 0; i--)
		{
			pair<double, double>p = res[i];
			ostringstream strs;
			strs << p.first << "\t" << p.second;
			file << strs.str() << "\n";
		}
		file.close();
	}
	else cout << "Search failed" << endl;
	nr.Clear();
	*/
	/*
	cout << "Start optimal..." << endl;
	//begin = clock();
	bool r = nr.StartRecord(4, 4, 4, 7, "treedata.dat", false);
	double len;
	if (r) nr.GetPath(res, len);
	//end = clock();
	//cout << "Search time: " << (end - begin) << " ms" << endl;
	if (r)
	{
		cout << "Path found with " << nr.iter_index << " iterations" << endl;
		cout << "Shortest path length: " << len << endl;
		cout << "Nodes: " << nr.GetIndex() - 1 << endl;

		ofstream file;
		file.open("pathdata.dat", ios::out | ios::trunc);
		//if (!file.is_open()) return false;
		//cout << "Path: " << endl;
		for (int i=res.size()-1;i>=0;i--)
		{
			pair<double, double>p = res[i];
			ostringstream strs;
			strs << p.first << "\t" << p.second;
			file << strs.str() << "\n";
		}
		file.close();
	}
	else cout << "Search failed" << endl;
	nr.Clear();
	*/
    #ifdef _WIN32
	std::cin.get();
    # endif 
    return 0;
}
