#include <math.h>
#include <time.h> 
#include <random>
#include <stdio.h> 
#include <stdlib.h> 
#include <algorithm>
#include <vector>
#include <limits>
#include <queue>
#include <list>

#define PI 3.1415926
using namespace std;

class RRT_Node {

private:

	RRT_Node* Parent;

	vector <double> Angles;
	
	vector<RRT_Node*> Children;

public:
	RRT_Node() { Parent = nullptr; };

	vector <double> getAngles() { return Angles; };

	RRT_Node* getParent() { return Parent; };

	vector<RRT_Node*> getChildren() { return Children; };

	void addChild(RRT_Node* ChildTobeAdded) { Children.push_back(ChildTobeAdded); };

	void setParent(RRT_Node* ParentTobeAdded) { Parent = ParentTobeAdded; };

	void setAngles(vector <double> AnglesTobeSet) { Angles = AnglesTobeSet; };

	void popChild(RRT_Node* popNode);

};



RRT_Node* getNearest(RRT_Node* currentNode, vector<double> sampleNode);

void DFS_RRT(RRT_Node* currentNode, vector<double> sampleNode, RRT_Node*& nearestNode, double& dist);

int extend(RRT_Node* currentNode, RRT_Node* lastNode, vector<double> q_rand, double eps, double* map,
	int x_size, int y_size, vector<double> endCoord_, double tol, int numChecks, bool RRTconnect, bool RRTstar);

int Interpolation(vector<double> q_rand, RRT_Node* q_nearest, RRT_Node* q_new, double eps, double* map, int x_size, int y_size, vector<double> goalAngles, double tol, int numChecks, bool RRTconnect,  bool RRTstar);

bool atGoal(vector<double> currentAngles, vector<double> goalAngles, double tol);


double euclideanDist(vector<double> Node1, vector<double> Node2);

double fRand(double fMin, double fMax);

int connect(RRT_Node* tree, RRT_Node* q_new1, RRT_Node* lastNode2,
	double eps, double* map, int x_size, int y_size, int numChecks, vector<double> goalAngles, double tol, bool RRTconnect, bool RRTstar);


vector<double> directExtend(vector<double> q_rand, RRT_Node* q_nearest, double eps, int numofinterpolations, int step, bool duringInterp);


class Node_PRM {

private:
	int ID;
	vector <int> connectedNodeID;
	vector <double> Angles;
	int Parent;
	double G;

	bool open;
	bool close;

public:

	Node_PRM();

	void openSetTrue() { open = true; };
	void openSetFalse() { open = false; };
	bool getOpen() { return open; };

	void closeSetTrue() { close = true; };
	void closeSetFalse() { close = false; };
	bool getclose() { return close; };
	vector <double> getAngles() { return this->Angles; };

	void setParent(int PtobeSet) { Parent = PtobeSet; };
	int getParent() { return Parent; };
	int getID() { return ID; };

	vector<int> getConnectedNode() { return connectedNodeID; };


	void setAngles(vector <double> AnglesTobeset) { Angles = AnglesTobeset; };

	void insertConnect(int con) { connectedNodeID.push_back(con); };

	void setID(int IDTobeSet) { ID = IDTobeSet; };



};

bool IsValidConnection_PRM(Node_PRM* node1, Node_PRM* node2, double* map,
	int x_size, int y_size, int numofinterpolations);




bool IsValidConnection_RRT(RRT_Node* node1, RRT_Node* node2, double* map, int x_size,
	int y_size, int numofinterpolations);

void DFS_RRTStar(RRT_Node* currentNode, RRT_Node* q_new, vector<RRT_Node*>& nearNodes,
	double dist);

void findNearNode(RRT_Node* currentNode, RRT_Node* q_new, vector<RRT_Node*>& nearNodes,
	double dist);

int extendStar(RRT_Node* currentNode, vector<RRT_Node*>& Nodes_atGoal,
	vector<double> q_rand, double eps, double* map, int x_size, int y_size,
	vector<double> q_end, double tol, double dist, int numofinterpolations);

double costOfNode(RRT_Node* currentNode);

