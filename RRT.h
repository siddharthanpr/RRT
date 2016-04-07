
#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;
using namespace std;
typedef double dReal;



class RRTNode
{
std::vector<double> _configuration;
RRTNode* parent;

public:

RRTNode*  getParent();
RRTNode();
~RRTNode();
void setParent(RRTNode* p);
RRTNode(std::vector<double> ele);
RRTNode(std::vector<double> ele,RRTNode* p);
std::vector<double> getConfig();

};


class NodeTree
{
std::vector<RRTNode*> _nodes;
public:
void add(RRTNode &ele);
void add(RRTNode* ele);
void delNode(int ele);
void delNode(RRTNode &ele);
void delBranch(RRTNode *start,RRTNode *endNode);
std::vector< std::vector<double> >  getNodes();
std::vector<RRTNode*> getTree();
std::vector<RRTNode*> getPathToStart(RRTNode* ele);
void display();
NodeTree();
NodeTree(RRTNode &ele);
};



bool inRegion(vector<double> inp);
void display(vector<float> inp);
void display(vector<dReal> inp);
vector<double> randomSample();
vector<double> subtractMul(vector<double> a,vector<double> &b, float scale);
vector<double> add(vector<double> a,vector<double> &b);
float norm(vector<double> a);
float euclidean(vector<double> a,vector<double> b,bool flag );
RRTNode* nearestNeighbor(std::vector<RRTNode*> tree, vector<double> inputConfig);
std::vector< RRTNode* > RRTplanner(OpenRAVE::EnvironmentBasePtr env, std::vector<double> goalConfig,float step,float goalBias);
float normalize_angle(float q);
float metric_weight[] = {1,1,1,1,1,1,1};
std::vector<double> lower;
std::vector<double> upper;
#include "kdtree.cpp"


#endif
