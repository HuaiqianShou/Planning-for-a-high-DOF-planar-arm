/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <time.h> 
#include <random>
#include <stdio.h> 
#include <stdlib.h> 
#include <algorithm>
#include <vector>
#include <stack>          // std::stack
#include <functional>
#include <queue>
#include <limits>
#include <unordered_map>

#include <iostream>
#include<cmath>

#include <chrono> 

#include "planner.h"


using namespace std::chrono;
/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10
using namespace std;
typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
        {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}



double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

bool sampleAtgoal(int currenttimes, int times) {

    if (currenttimes / times > 0 && currenttimes % times == 0) {
        return true;
    }
    else {
        return false;
    }

}

bool atGoal(vector<double> currentAngles, vector<double> goalAngles, double tol) {

    for (int i = 0; i < goalAngles.size(); i++) {

        if ((currentAngles[i] - goalAngles[i]) > tol || (currentAngles[i] - goalAngles[i]) < -tol)

            return false;
    }

    return true;
}


double euclideanDist(vector<double> Node1, vector<double> Node2) {
    double distance = 0;
    for (int i = 0; i < Node1.size(); i++) {
        distance += min((Node1[i] - Node2[i]) * (Node1[i] - Node2[i]),
            (2 * PI - fabs(Node1[i] - Node2[i])) * (2 * PI - fabs(Node1[i] - Node2[i])));
        //distance = atan2(sin(Node1[i] - Node2[i]), cos(Node1[i] - Node2[i]));
    }
    distance = sqrt(distance);
    return distance;
}

void DFS_RRT(RRT_Node* currentNode, vector<double> sampleNode, RRT_Node*& nearestNode, double& dist) {

    if (!currentNode->getChildren().empty()) {

        for (auto i : currentNode->getChildren()) {

            if (euclideanDist(i->getAngles(), sampleNode) < dist)
            {
                dist = euclideanDist(i->getAngles(), sampleNode);
                nearestNode = i;
            }
            DFS_RRT(i, sampleNode, nearestNode, dist);
        }
        return;
    }
    else {

        return;
    }
}



RRT_Node* getNearest(RRT_Node* currentNode,vector<double> sampleNode) {

    double dist = euclideanDist(currentNode->getAngles(), sampleNode);

    RRT_Node* nearestNode = currentNode;

    DFS_RRT(currentNode, sampleNode, nearestNode, dist);

    return nearestNode;
}

vector<double> directExtend(vector<double> q_rand, RRT_Node* q_nearest, double eps, int numofinterpolations, int step, bool duringInterp) {

    double distanceTemp = euclideanDist(q_nearest->getAngles(), q_rand);
    double ratio_temp = min(distanceTemp, eps) / distanceTemp;
    if (duringInterp) {
        ratio_temp = (double)(step + 1)/ numofinterpolations;
    }
    
    vector<double> q_direct;

    for (int j = 0; j < q_nearest->getAngles().size(); j++) {

        if (q_rand[j] - q_nearest->getAngles()[j] >= PI) {

            q_direct.push_back(q_nearest->getAngles()[j] + ratio_temp * (q_rand[j]
                - q_nearest->getAngles()[j] - 2 * PI));
        }
        else if (q_rand[j] - q_nearest->getAngles()[j] <= -PI) {

            q_direct.push_back(q_nearest->getAngles()[j] + ratio_temp * (q_rand[j]
                - q_nearest->getAngles()[j] + 2 * PI));
        }
        else {

            q_direct.push_back(q_nearest->getAngles()[j] + ratio_temp * (q_rand[j]
                - q_nearest->getAngles()[j]));
        }
    }

    return q_direct;

}

int Interpolation(vector<double> q_rand, RRT_Node* q_nearest, RRT_Node* q_new, double eps, double* map, int x_size, int y_size, vector<double> goalAngles, double tol, int numofinterpolations,bool RRTconnect, bool RRTstar) {

    double distanceTemp = euclideanDist(q_nearest->getAngles(), q_rand);
    bool duringInterp = false;
    vector<double> q_direct = directExtend(q_rand, q_nearest, eps, numofinterpolations, 3, duringInterp);
    vector<double> q_interp_temp = q_nearest->getAngles();
    duringInterp = true;

    for (int i = 0; i < numofinterpolations; i++) {
        
        vector<double> q_interp_current = directExtend(q_direct ,q_nearest, eps, numofinterpolations, i, duringInterp);

        if (IsValidArmConfiguration(q_interp_current.data(), q_interp_current.size(), map, x_size, y_size) == 0) {

            if (i == 0) {
                if (RRTstar) {
                    delete q_new;
                    return 2;
                }
                else {
                    return 2;
                }
            }
            else {
                q_new->setAngles(q_interp_temp);
                return 1;
            }
        }

        q_interp_temp = q_interp_current;

        if ((!RRTconnect) && atGoal(q_interp_current, goalAngles, tol)) {

            q_new->setAngles(q_interp_current);
            return 3;
        }
    }

    if (eps >= distanceTemp) {

        q_new->setAngles(q_rand);
        return 0;
    }
    else {

        q_new->setAngles(q_direct);
        return 1;
    }
}

int extend(RRT_Node* currentNode, RRT_Node* lastNode, vector<double> q_rand, double eps, double* map,
    int x_size, int y_size, vector<double> goalAngles, double tol, int numofinterpolations, bool RRTconnect,bool RRTstar) {

    RRT_Node* nearestNode = getNearest(currentNode, q_rand);  
    int interpResult = -1;
    RRT_Node* newNode = new RRT_Node;
    
    if (RRTconnect) {
        interpResult = Interpolation(q_rand, nearestNode, lastNode, eps, map, x_size, y_size, goalAngles, tol, numofinterpolations, RRTconnect, RRTstar);
    }
    else {
        interpResult = Interpolation(q_rand, nearestNode, newNode, eps, map, x_size, y_size, goalAngles, tol, numofinterpolations, RRTconnect, RRTstar);
    }

    switch (interpResult) {
    case 0:
        if (RRTconnect) {
            lastNode->setParent(nearestNode);
            nearestNode->addChild(lastNode);
        }
        else {
            newNode->setParent(nearestNode);
            nearestNode->addChild(newNode);
        }
        return 0;
    case 1:
        if (RRTconnect) {
            lastNode->setParent(nearestNode);
            nearestNode->addChild(lastNode);
        }
        else {
            newNode->setParent(nearestNode);
            nearestNode->addChild(newNode);
        }
        return 1;
    case 2:
        if (RRTconnect) {
            delete lastNode;
        }
        else {
           delete newNode;
        }
        return 2;
    case 3:
        lastNode->setAngles(newNode->getAngles());
        lastNode->setParent(nearestNode);
        nearestNode->addChild(lastNode);
        return 3;
    }
}






static void plannerRRT(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
    //no plan by default
    *plan = NULL;
    *planlength = 0;

    // define parameters
    double eps = numofDOFs/25.0;
    double tol = 1.5 * PI / 180;
    int numofinterpolations = int(200 * eps);   
    bool foundGoal = false;

    int numofNodes = 0;
    bool RRTconnect = 0;
    bool RRTstar = 0;
    int timeTosampleAtgoal = 5;
    int numofsamples = 0;
    // initialize the start and end node and reverse them
    vector<double> armAngles_Start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> armAngles_Goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
    vector<double> q_start, q_end;
    q_start = armAngles_Goal;
    q_end = armAngles_Start;

    if (IsValidArmConfiguration(armAngles_Start.data(), numofDOFs, map, x_size, y_size) == 1
        && IsValidArmConfiguration(armAngles_Goal.data(), numofDOFs, map, x_size, y_size) == 1) {


        auto start = high_resolution_clock::now();

        RRT_Node* currentNode = new RRT_Node;
        currentNode->setParent(nullptr);
        currentNode->setAngles(q_start);

        RRT_Node* lastNode = new RRT_Node;
        lastNode->setParent(nullptr);

        while (!foundGoal) {
            vector<double> q_rand;

            if (sampleAtgoal(numofsamples, timeTosampleAtgoal)) {
                for (int i = 0; i < numofDOFs; i++) {

                    double temp = fRand(q_end[i] - tol/10.0, q_end[i] + tol/10.0);
                    q_rand.push_back(temp);
                }
            }
            else {
                for (int i = 0; i < numofDOFs; i++) {

                    double temp = fRand(0.0, 6.283);
                    q_rand.push_back(temp);
                }

            }
            numofsamples++;
            if (IsValidArmConfiguration(q_rand.data(), numofDOFs, map, x_size, y_size) == 1) {
                int resultExtend = extend(currentNode, lastNode, q_rand, eps, map, x_size, y_size, q_end,
                    tol, numofinterpolations,RRTconnect,RRTstar);

                //cout << resultExtend << endl;

                if (resultExtend != 2)
                    numofNodes++;

                if (resultExtend == 3)
                    foundGoal = true;
            }
        }
        queue< vector<double> > Path;
        if (lastNode->getParent() != nullptr) {

            Path.push(lastNode->getAngles());
            RRT_Node* lastNode_temp = lastNode;

            while (lastNode_temp->getParent() != nullptr) {

                lastNode_temp = lastNode_temp->getParent();
                Path.push(lastNode_temp->getAngles());
            }
        }
        int sizePath = Path.size();
        *planlength = sizePath;
        *plan = (double**)malloc(sizePath * sizeof(double*));
        int firstinvalidconf = 1;
        for (int i = 0; i < sizePath; i++) {

            if (Path.empty()) {

                break;
            }
            (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

            for (int j = 0; j < numofDOFs; j++) {

                (*plan)[i][j] = Path.front()[j];
            }
            if (IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) == 0 && firstinvalidconf)
            {
                firstinvalidconf = 1;
                printf("ERROR: Invalid arm configuration!!!\n");
            }

            Path.pop();

        }

        if (currentNode != nullptr)
            delete currentNode;

        if (lastNode != nullptr)
            delete lastNode;

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        double time_count = double(duration.count() / 1000000.0);
        mexPrintf("\n      RRT  results    \n\n");
        mexPrintf("Planning time: %fs \n", time_count);
        mexPrintf("Number of nodes in the tree: %d \n", numofNodes);
        mexEvalString("drawnow");
    }

    else {

        mexPrintf("Failed start or goal angles invalid \n");
        mexEvalString("drawnow");
    }
    return;
}

int connect(RRT_Node* tree, RRT_Node* q_new1, RRT_Node* lastNode2,
    double eps, double* map, int x_size, int y_size, int numofinterpolations, vector<double> goalAngles, double tol, bool RRTconnect, bool RRTstar) {
    int result;
    do {

        RRT_Node* q_new2 = new RRT_Node;
        result = extend(tree, q_new2, q_new1->getAngles(),  eps, map, x_size, y_size,  goalAngles, tol, numofinterpolations,RRTconnect,RRTstar);
        if (result == 0) {

            *lastNode2 = *q_new2;
            return 0;
        }
        else if (result == 2)
            return 2;

    } while (result == 1);

    return result;
}



static void RRT_connect(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{

    *plan = NULL;
    *planlength = 0;


    double eps = (numofDOFs)/25.0;
    int numofinterpolations = int(200 * eps);


    bool treeConnected = false;
    int numofNodes = 0;
    double tol = 0;
    bool RRTconnect = 1;
    bool RRTstar = 0;

    vector<double> q_start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> q_end(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    if (IsValidArmConfiguration(q_start.data(), numofDOFs, map, x_size, y_size) == 1
        && IsValidArmConfiguration(q_end.data(), numofDOFs, map, x_size, y_size) == 1) {

        auto start = high_resolution_clock::now();
        // initialize both trees
        RRT_Node* tree1 = new RRT_Node;
        tree1->setParent(nullptr);
        tree1->setAngles(q_start);

        RRT_Node* tree2 = new RRT_Node;
        tree2->setParent(nullptr);
        tree2->setAngles(q_end);

        RRT_Node* lastNode1 = new RRT_Node;
        RRT_Node* lastNode2 = new RRT_Node;

        while (!treeConnected) {


            vector<double> q_rand;

            for (int i = 0; i < numofDOFs; i++) {

                double temp = fRand(0.0, 6.283);
                q_rand.push_back(temp);

            }

            if (IsValidArmConfiguration(q_rand.data(), numofDOFs, map, x_size, y_size) == 1) {

                RRT_Node* newNode1 = new RRT_Node;

                if (extend(tree1, newNode1, q_rand,  eps, map, x_size, y_size, q_end,tol, numofinterpolations,RRTconnect,RRTstar) != 2) {
                    numofNodes++;
                    //cout << numofNodes << endl;
                    int resultConnect = connect(tree2, newNode1, lastNode2, eps, map, x_size, y_size, numofinterpolations,q_end, tol, RRTconnect, RRTstar);
                    if (resultConnect != 2)
                        numofNodes++;
                        //cout << numofNodes << endl;
                    if (resultConnect == 0) {
                        treeConnected = true;
                        lastNode1 = newNode1;                      
                        break;
                    }
                }
                RRT_Node temp = *tree1;
                *tree1 = *tree2;
                *tree2 = temp;
            }
        }
        list< vector<double> > Path; 
        bool forward = (tree1->getAngles() == q_start);
        if (lastNode1->getParent() != nullptr) {
            if (forward) {
                Path.push_front(lastNode1->getAngles());
                RRT_Node* lastNode_temp = lastNode1;
                while (lastNode_temp->getParent() != nullptr) {
                    lastNode_temp = lastNode_temp->getParent();
                    Path.push_front(lastNode_temp->getAngles());
                }
            }
            else {
                Path.push_back(lastNode1->getAngles());
                RRT_Node* lastNode_temp = lastNode1;
                while (lastNode_temp->getParent() != nullptr) {
                    lastNode_temp = lastNode_temp->getParent();
                    Path.push_back(lastNode_temp->getAngles());
                }
            }            
        }
        if (lastNode2->getParent() != nullptr) {
            if (forward) {
                Path.push_back(lastNode2->getAngles());
                RRT_Node* lastNode_temp = lastNode2;
                while (lastNode_temp->getParent() != nullptr) {
                    lastNode_temp = lastNode_temp->getParent();
                    Path.push_back(lastNode_temp->getAngles());
                }
            }
            else {
                Path.push_front(lastNode2->getAngles());
                RRT_Node* lastNode_temp = lastNode2;
                while (lastNode_temp->getParent() != nullptr) {
                    lastNode_temp = lastNode_temp->getParent();
                    Path.push_front(lastNode_temp->getAngles());
                }
            }           
        }
        Path.pop_front(); 
        Path.pop_back();
        Path.push_front(q_start);
        Path.push_back(q_end);

        int sizePath = Path.size();
        *planlength = sizePath;

        *plan = (double**)malloc(sizePath * sizeof(double*));
        for (int i = 0; i < sizePath; i++) {
            if (Path.empty()) {
                break;
            }
            (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
            int firstinvalidconf = 1;
            for (int j = 0; j < numofDOFs; j++) {
                (*plan)[i][j] = Path.front()[j];               
            }
            if (IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) == 0 && firstinvalidconf)
            {
                firstinvalidconf = 1;
                printf("ERROR: Invalid arm configuration!!!\n");
            }
            Path.pop_front();
        }
        if (tree1 != nullptr)
            delete tree1;
        if (tree2 != nullptr)
            delete tree2;
        if (lastNode1 != nullptr)
            delete lastNode1;
        if (lastNode2 != nullptr)
            delete lastNode2;
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        double time_count = double(duration.count() / 1000000.0);
        mexPrintf("\n      RRT connect  results       \n\n");
        mexPrintf("Planning time: %fs \n", time_count);
        mexPrintf("Number of nodes in the tree: %d \n", numofNodes);
        mexEvalString("drawnow");
    }
    else {

        mexPrintf("Failed start or goal angles invalid \n");
        mexEvalString("drawnow");
    }
}


void RRT_Node::popChild(RRT_Node* popNode) {


    for (auto it = Children.begin(); it != Children.end(); it++) {

        if (popNode->getAngles() == (*it)->getAngles()) {		
            Children.erase(it);
            return;
        }
    }
}




RRT_Node* getNearest(vector<double> q_rand, RRT_Node* currentNode) {


    double min_val = euclideanDist(currentNode->getAngles(), q_rand);
    RRT_Node* nearest = currentNode;
    DFS_RRT(currentNode, q_rand, nearest, min_val);


    return nearest;
}

bool IsValidConnection_RRT(RRT_Node* node1, RRT_Node* node2, double* map, int x_size,
    int y_size, int numofinterpolations) {


    for (int i = 0; i < numofinterpolations; i++) {

        vector<double> q_direct;

        for (int j = 0; j < node1->getAngles().size(); j++) {

            if (node2->getAngles()[j] - node1->getAngles()[j] >= PI) {
                q_direct.push_back(node1->getAngles()[j] + (double)(i + 1) *
                    (node2->getAngles()[j] - node1->getAngles()[j] - 2 * PI) / numofinterpolations);
            }
            else if (node2->getAngles()[j] - node1->getAngles()[j] <= -PI) {
                q_direct.push_back(node1->getAngles()[j] + (double)(i + 1) *
                    (node2->getAngles()[j] - node1->getAngles()[j] + 2 * PI) / numofinterpolations);
            }
            else {
                q_direct.push_back(node1->getAngles()[j] + (double)(i + 1) *
                    (node2->getAngles()[j] - node1->getAngles()[j]) / numofinterpolations);
            }
        }

        if (IsValidArmConfiguration(q_direct.data(), node1->getAngles().size(), map, x_size, y_size) == 0) {

            return false;
        }
    }

    return true;
}


void DFS_RRTStar(RRT_Node* currentNode, RRT_Node* q_new, vector<RRT_Node*>& nearNodes,
    double dist) {

    if (!currentNode->getChildren().empty()) {

        for (auto i : currentNode->getChildren()) {

            if (euclideanDist(i->getAngles(), q_new->getAngles()) < dist) {

                nearNodes.push_back(i);
            }

            DFS_RRTStar(i, q_new, nearNodes, dist);
        }

        return;
    }
    else {

        return;
    }
}

void findNearNode(RRT_Node* currentNode, RRT_Node* q_new, vector<RRT_Node*>& nearNodes,
    double dist) {

    if (euclideanDist(currentNode->getAngles(), q_new->getAngles()) < dist) {

        nearNodes.push_back(currentNode);
    }

    DFS_RRTStar(currentNode, q_new, nearNodes, dist);
    return;
}


int extendStar(RRT_Node* currentNode, vector<RRT_Node*>& Nodes_atGoal,
    vector<double> q_rand, double eps, double* map, int x_size, int y_size,
    vector<double> q_end, double tol, double dist, int numofinterpolations) {

    RRT_Node* nearestNode = getNearest(currentNode,q_rand);

    RRT_Node* newNode = new RRT_Node;

    bool RRTconnect = 0;
    bool RRTstar = 1;
    int steerKey = Interpolation(q_rand, nearestNode, newNode, eps, map, x_size, y_size,
        q_end, tol, numofinterpolations,RRTconnect,RRTstar);

    if (steerKey == 2) {

        return steerKey;
    }

    RRT_Node* minNode = nearestNode;

    int minIndex;

    vector<RRT_Node*> nearNodes;

    findNearNode(currentNode, newNode, nearNodes, dist);

    vector<bool> freeNearNodes;

    for (int i = 0; i < nearNodes.size(); i++) {

        if (IsValidConnection_RRT(nearNodes[i], newNode, map, x_size, y_size, numofinterpolations)) {

            freeNearNodes.push_back(true);
            double costNew = costOfNode(minNode) +
                euclideanDist(newNode->getAngles(), minNode->getAngles());

            double costTemp = costOfNode(nearNodes[i]) + euclideanDist(newNode->getAngles(),
                nearNodes[i]->getAngles());


            if (costTemp < costNew) {

                minNode = nearNodes[i];
                minIndex = i;
            }
        }
        else {
            freeNearNodes.push_back(false);
        }
    }

    minNode->addChild(newNode);
    newNode->setParent(minNode);

    for (int i = 0; i < nearNodes.size(); i++) {
        if (i != minIndex) {
            if ((freeNearNodes[i] == true) && (costOfNode(nearNodes[i]) > costOfNode(newNode) +
                euclideanDist(newNode->getAngles(), nearNodes[i]->getAngles()))) {
                RRT_Node* tempParent = nearNodes[i]->getParent();
                newNode->setParent(tempParent);
                tempParent->addChild(newNode);
                tempParent->popChild(nearNodes[i]);
            }
        }
    }
    if (steerKey == 3) {

        Nodes_atGoal.push_back(newNode);
        return steerKey;
    }

    return steerKey;

}
double costOfNode(RRT_Node* currentNode) {

    double cost = 0;

    RRT_Node* temp = currentNode;
    while (temp->getParent() != nullptr) {

        cost += euclideanDist(temp->getAngles(), temp->getParent()->getAngles());
        temp = temp->getParent();
    }

    return cost;
}




static void plannerRRT_star(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
    //no plan by default
    *plan = NULL;
    *planlength = 0;


    double eps = numofDOFs / 25.0;
    double tol = 1.5 * PI / 180;
    int N_star = 500;
    int numofinterpolations = floor(200 * eps);
    double nearDist = 0.6;


    auto start1 = high_resolution_clock::now();
    int timeTosampleAtgoal = 5;
    int numofsamples = 0;
    int numofNodes = 0;

    vector<double> armAngles_Start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> armAngles_Goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
    vector<RRT_Node*> goalNodes;
    vector<double> q_start;
    vector<double> q_end;
    q_start = armAngles_Goal;
    q_end = armAngles_Start;

    if (IsValidArmConfiguration(armAngles_Start.data(), numofDOFs, map, x_size, y_size) == 1
        && IsValidArmConfiguration(armAngles_Goal.data(), numofDOFs, map, x_size, y_size) == 1) {

        auto start = high_resolution_clock::now();

        RRT_Node* currentNode = new RRT_Node;
        currentNode->setParent(nullptr);
        currentNode->setAngles(q_start);

        while (numofNodes < N_star || goalNodes.empty()) {
            vector<double> q_rand;
            if (sampleAtgoal(numofsamples, timeTosampleAtgoal)) {
                for (int i = 0; i < numofDOFs; i++) {
                    double temp = fRand(q_end[i] - tol/10.0, q_end[i] + tol/10.0);
                    q_rand.push_back(temp);
                }
            }
            else {
                for (int i = 0; i < numofDOFs; i++) {
                    double temp = fRand(0.0, 6.283);
                    q_rand.push_back(temp);
                }
            }

            numofsamples++;

            if (IsValidArmConfiguration(q_rand.data(), numofDOFs, map, x_size, y_size) == 1) {
                int result = extendStar(currentNode, goalNodes, q_rand, eps, map, x_size, y_size,
                    q_end, tol, nearDist, numofinterpolations);
                if (result != 2)
                    numofNodes++;
                //printf("numofNodes is %d\n", numofNodes);

            }
        }

        queue< vector<double> > Path;
        if (goalNodes[0]->getParent() != nullptr) {

            Path.push(goalNodes[0]->getAngles());
            RRT_Node* temp = goalNodes[0];

            while (temp->getParent() != nullptr) {

                temp = temp->getParent();
                Path.push(temp->getAngles());
            }
        }
        int sizePath = Path.size();
        *planlength = sizePath;

        *plan = (double**)malloc(sizePath * sizeof(double*));
        int firstinvalidconf = 1;

        for (int i = 0; i < sizePath; i++) {
            if (Path.empty()) {
                break;
            }
            (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
            for (int j = 0; j < numofDOFs; j++) {
                (*plan)[i][j] = Path.front()[j];
            }
            if (IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) == 0 && firstinvalidconf)
            {
                firstinvalidconf = 1;
                printf("ERROR: Invalid arm configuration!!!\n");
            }
            Path.pop();
        }


        if (currentNode != nullptr)
            delete currentNode;



        auto stop = high_resolution_clock::now();
        auto time = duration_cast<microseconds>(stop - start);
        double time_count = double(time.count() / 1000000.0);
        mexPrintf("\n        RRT_star results        \n\n");
        mexPrintf("Planning time: %fs \n", time_count);
        mexPrintf("Number of nodes in the tree: %d \n", numofNodes);
        mexEvalString("drawnow");

    }
    else {

        mexPrintf("Failed start or goal angles invalid  \n");
        mexEvalString("drawnow");
    }

    return;
}

Node_PRM::Node_PRM() : open(false), close(false) {}

bool IsValidConnection_PRM(Node_PRM* node1, Node_PRM* node2, double* map, int x_size, int y_size, int numofinterpolations) {

    for (int i = 0; i < numofinterpolations; i++) {

        vector<double> q_direct;
        for (int j = 0; j < node1->getAngles().size(); j++) {

            if (node2->getAngles()[j] - node1->getAngles()[j] >= PI) {

                q_direct.push_back(node1->getAngles()[j] + (double)(i + 1) *
                    (node2->getAngles()[j] - node1->getAngles()[j] - 2 * PI) / numofinterpolations);
                // printf("greater than pi : %d\n", j);
            }
            else if (node2->getAngles()[j] - node1->getAngles()[j] <= -PI) {

                q_direct.push_back(node1->getAngles()[j] + (double)(i + 1) *
                    (node2->getAngles()[j] - node1->getAngles()[j] + 2 * PI) / numofinterpolations);
                // printf("lesser than -pi : %d \n", j);
            }
            else {

                q_direct.push_back(node1->getAngles()[j] + (double)(i + 1) *
                    (node2->getAngles()[j] - node1->getAngles()[j]) / numofinterpolations);
                // printf("in between : %d\n", j);
            }
        }

        if (IsValidArmConfiguration(q_direct.data(), node1->getAngles().size(), map, x_size, y_size) == 0) {

            return false;
        }
    }

    return true;
}

static void plannerPRM(double* map, int x_size, int y_size, double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength)
{
    //no plan by default
    *plan = NULL;
    *planlength = 0;

    // parameters



    double eps = numofDOFs / 20.0;
    double tol = 2.0 * PI / 180;
    int numofinterpolations = int(200 * eps);
    int numofsamples;
    
    numofsamples = 1000 + 200 * numofDOFs;

    vector<double> q_start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> q_end(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    unordered_map<int, Node_PRM*> nodesMap;
    int k;
    int numofnodeCount = 0;

    if (IsValidArmConfiguration(q_start.data(), numofDOFs, map, x_size, y_size) == 1
        && IsValidArmConfiguration(q_end.data(), numofDOFs, map, x_size, y_size) == 1){ 


        auto start = high_resolution_clock::now();
        while (numofnodeCount < numofsamples) {

            Node_PRM* newNode = new Node_PRM();

            bool Valid_q_rand = false;


            do {
                vector <double> q_rand;
                for (int m = 0; m < numofDOFs; m++) {
                    double temp = fRand(0.0, 6.283);
                    q_rand.push_back(temp);
                }
                if (IsValidArmConfiguration(q_rand.data(), numofDOFs, map, x_size, y_size) == 1) {

                    Valid_q_rand = true;
                    newNode->setID(numofnodeCount);
                    newNode->setAngles(q_rand);

                }
            } while (!Valid_q_rand);

            vector<pair<double, int>> distances;
            for (int i = 0; i < numofnodeCount; i++) {
                //cout << i << endl;
                double dist = euclideanDist(newNode->getAngles(), nodesMap.at(i)->getAngles());
                distances.push_back(make_pair(dist, i));

            }
            sort(distances.begin(), distances.end());
            int connectCount = 0;
            k = 0;
            while (connectCount < 3 && k < numofnodeCount) {
                //cout << "fuck" << endl;
                if (IsValidConnection_PRM(newNode, nodesMap.at(distances[k].second), map, x_size, y_size, numofinterpolations) == 1) {

                    newNode->insertConnect(distances[k].second);
                    nodesMap.at(distances[k].second)->insertConnect(numofnodeCount);
                    connectCount++;
                }
                k++;
            }
            nodesMap[numofnodeCount] = newNode;
            numofnodeCount++;
        }

        Node_PRM* startNode = new Node_PRM();
        Node_PRM* endNode = new Node_PRM();

        startNode->setAngles(q_start);
        startNode->setID(-1);
        endNode->setAngles(q_end);
        endNode->setID(-2);

        //cout << "hello" << endl;

        vector<pair<double, int>> distances = {};

        //cout << "hello2" << endl;

        for (int i = 0; i < numofsamples; i++) {
            //cout << i << endl;
            double dist = euclideanDist(startNode->getAngles(), nodesMap.at(i)->getAngles());
            distances.push_back(make_pair(dist, i));
            //cout << i << endl;

        }
        sort(distances.begin(), distances.end());
        //cout << "nihao" << endl;
        bool isConnected = false;
        k = 0;
        //printf("a\n");
        while (!isConnected && k < numofsamples) {

            if (IsValidConnection_PRM(startNode, nodesMap.at(distances[k].second), map, x_size, y_size, numofinterpolations) == 1) {
                startNode->insertConnect(distances[k].second);
                nodesMap.at(distances[k].second)->insertConnect(-1);
                isConnected = true;
            }
            k++;
        }
        //printf("b\n");
        startNode->setParent(-1);
        nodesMap[-1] = startNode;

        endNode->setID(-2);
        distances.clear();

        for (int i = 0; i < numofsamples; i++) {
            double distance = euclideanDist(endNode->getAngles(), nodesMap.at(i)->getAngles());
            distances.push_back(make_pair(distance, i));
        }
        sort(distances.begin(), distances.end());

        isConnected = false;
        k = 0;
        while (!isConnected && k < numofsamples) {
            if (IsValidConnection_PRM(endNode, nodesMap.at(distances[k].second), map, x_size, y_size, numofinterpolations) == 1) {
                endNode->insertConnect(distances[k].second);
                nodesMap.at(distances[k].second)->insertConnect(-2);
                isConnected = true;
            }
            k++;
        }
        nodesMap[-2] = endNode;

        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> OpenList;
        stack<int, vector<int>> Path;

        OpenList.emplace(make_pair(0.0, -1));
        while (!OpenList.empty()) {

            pair<double, int> temp = OpenList.top();
            int tempID = temp.second;
            int G = temp.first;
            OpenList.pop();
            nodesMap.at(tempID)->openSetTrue();
            nodesMap.at(tempID)->closeSetTrue();

            for (int i = 0; i < nodesMap.at(tempID)->getConnectedNode().size(); i++) {

                int nextID = nodesMap.at(tempID)->getConnectedNode()[i];
                if (nextID == -2) {
                    nodesMap.at(nextID)->setParent(tempID);

                    int currenttempID = -2;
                    int j = 0;
                    while (currenttempID != -1) {

                        Path.push(currenttempID);
                        currenttempID = nodesMap.at(currenttempID)->getParent();
                        j++;
                    }
                    Path.push(-1);
                    //printf("end: ");
                    for (int z = 0; z < numofDOFs; z++)
                    {
                        //printf("%f ", nodesMap.at(-2)->getAngles()[z]);
                    }
                    //printf("\n");

                    *planlength = j + 1;
                    *plan = (double**)malloc((j + 1) * sizeof(double*));
                    //printf("jiesi: %d\n", (j+1));
                    for (int k = 0; k < j + 1; k++) {
                        //printf("fatiao\n");
                        (*plan)[k] = (double*)malloc(numofDOFs * sizeof(double));
                        //printf("index: %d\n",Path.top());
                        for (int l = 0; l < numofDOFs; l++) {
                            //printf("wowuyu\n");
                            (*plan)[k][l] = nodesMap.at(Path.top())->getAngles()[l];

                        }
                        Path.pop();


                    }
                    auto stop = high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>(stop - start);
                    double time_count = double(duration.count() / 1000000.0);

                    mexPrintf("\n          PRM  results          \n\n");
                    mexPrintf("Planning time: %fs \n", time_count);
                    mexPrintf("Number of nodes in the tree: %d \n", numofnodeCount);
                    mexEvalString("drawnow");
                    return;

                }
                if (!nodesMap.at(nextID)->getclose() && !nodesMap.at(nextID)->getOpen()) {

                    //printf("shit shit!\n");
                    double newG = G + euclideanDist(nodesMap.at(nextID)->getAngles(), nodesMap.at(tempID)->getAngles());
                    nodesMap.at(nextID)->setParent(tempID);
                    OpenList.emplace(make_pair(newG, nextID));
                    nodesMap.at(nextID)->openSetTrue();

                }
            }

        }  

    
    }
    else {
        mexPrintf("Failed start or goal angles invalid \n");
        mexEvalString("drawnow");

    }

    return;    
}



//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    //you can may be call the corresponding planner function here
    //if (planner_id == RRT)
    //{
    //    plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    //}
    
    //dummy planner which only computes interpolated path
    //planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    if (planner_id == 0)
    {
        plannerRRT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    else if (planner_id == 1) {
        RRT_connect(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    else if (planner_id == 2) {
        plannerRRT_star(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    else if (planner_id == 3) {

        plannerPRM(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    }
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





