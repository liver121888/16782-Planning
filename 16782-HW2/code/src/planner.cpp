/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <stack>
#include <deque>
#include <queue>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex>	 // For regex and split logic
#include <iostream>	 // cout, endl
#include <fstream>	 // For reading/writing files
#include <assert.h>

/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTCONNECT 1
#define RRTSTAR 2
#define PRM 3

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y * XSIZE + X)

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

// the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif

// some potentially helpful imports
using std::array;
using std::cout;
using std::deque;
using std::endl;
using std::make_tuple;
using std::runtime_error;
using std::stack;
using std::string;
using std::tie;
using std::tuple;
using std::vector;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief
/// @param filepath
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath)
{
  std::FILE* f = fopen(filepath.c_str(), "r");
  if (f)
  {
  }
  else
  {
	printf("Opening file failed! \n");
	throw runtime_error("Opening map file failed!");
  }
  int height, width;
  if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2)
  {
	throw runtime_error("Invalid loadMap parsing map metadata");
  }

  ////// go through file and add to m_occupancy
  double* map = new double[height * width];

  double cx, cy, cz;
  for (int y = 0; y < height; y++)
  {
	for (int x = 0; x < width; x++)
	{
	  char c;
	  do
	  {
		if (fscanf(f, "%c", &c) != 1)
		{
		  throw runtime_error("Invalid parsing individual map data");
		}
	  } while (isspace(c));
	  if (!(c == '0'))
	  {
		map[y + x * width] = 1;	 // Note transposed from visual
	  }
	  else
	  {
		map[y + x * width] = 0;
	  }
	}
  }
  fclose(f);
  return make_tuple(map, width, height);
}

// splits string based on deliminator
vector<string> split(const string& str, const string& delim)
{
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
  const std::regex ws_re(delim);
  return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}

double* doubleArrayFromString(string str)
{
  vector<string> vals = split(str, ",");
  double* ans = new double[vals.size()];
  for (int i = 0; i < vals.size(); ++i)
  {
	ans[i] = std::stod(vals[i]);
  }
  return ans;
}

bool equalDoubleArrays(double* v1, double* v2, int size)
{
  for (int i = 0; i < size; ++i)
  {
	if (abs(v1[i] - v2[i]) > 1e-3)
	{
	  // cout << endl;
	  return false;
	}
  }
  return true;
}

typedef struct
{
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

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int* pY, int x_size, int y_size)
{
  double cellsize = 1.0;
  // take the nearest cell
  *pX = (int)(x / (double)(cellsize));
  if (x < 0)
	*pX = 0;
  if (*pX >= x_size)
	*pX = x_size - 1;

  *pY = (int)(y / (double)(cellsize));
  if (y < 0)
	*pY = 0;
  if (*pY >= y_size)
	*pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t* params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
	(params->UsingYIndex)++;

  if (params->UsingYIndex)
  {
	params->Y1 = p1x;
	params->X1 = p1y;
	params->Y2 = p2x;
	params->X2 = p2y;
  }
  else
  {
	params->X1 = p1x;
	params->Y1 = p1y;
	params->X2 = p2x;
	params->Y2 = p2y;
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

  params->DeltaX = params->X2 - params->X1;
  params->DeltaY = params->Y2 - params->Y1;

  params->IncrE = 2 * params->DeltaY * params->Increment;
  params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
  params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t* params, int* x, int* y)
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

int get_next_point(bresenham_param_t* params)
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

int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map, int x_size, int y_size)
{
  bresenham_param_t params;
  int nX, nY;
  short unsigned int nX0, nY0, nX1, nY1;

  // printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

  // make sure the line segment is inside the environment
  if (x0 < 0 || x0 >= x_size || x1 < 0 || x1 >= x_size || y0 < 0 || y0 >= y_size || y1 < 0 || y1 >= y_size)
	return 0;

  ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
  ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

  // printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

  // iterate through the points on the segment
  get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
  do
  {
	get_current_point(&params, &nX, &nY);
	if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1)
	  return 0;
  } while (get_next_point(&params));

  return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size)
{
  double x0, y0, x1, y1;
  int i;

  // iterate through all the links starting with the base
  x1 = ((double)x_size) / 2.0;
  y1 = 0;
  for (i = 0; i < numofDOFs; i++)
  {
	// compute the corresponding line segment
	x0 = x1;
	y0 = y1;
	x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles[i]);
	y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles[i]);

	// check the validity of the corresponding line segment
	// line segment means a link
	if (!IsValidLineSegment(x0, y0, x1, y1, map, x_size, y_size))
	  return 0;
  }
  return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MY FUNCTIONS                                                       //
//                                                                                                                   //
//*******************************************************************************************************************//

bool roughlyEqualDoubleArrays(double* v1, double* v2, int size)
{
  double thresh = 1e-2;
  double curSum = 0;
  for (int i = 0; i < size; ++i)
  {
	curSum += abs(v1[i] - v2[i]);
  }
  // curSum = sqrt(curSum);
  if (curSum < thresh)
  {
	return true;
  }
  return false;
}

double costMax = 100000.0;

struct vertex
{
  vector<double> angles;
  vertex* parent;

  // Constructor to initialize the vertex
  vertex(vector<double> angles, int numofDOFs, vertex* parent = nullptr)
  {
	this->parent = parent;
	this->angles.resize(numofDOFs);
	for (size_t i = 0; i < numofDOFs; i++)
	{
	  this->angles[i] = angles[i];
	}
  }

  // for RRTStar
  static double calculateCost(vertex* q1, vertex* q2)
  {
	double cost = 0.0;
	for (size_t i = 0; i < q1->angles.size(); i++)
	{
	  double dist = q1->angles[i] - q2->angles[i];
	  cost += dist * dist;
	}
	return sqrt(cost);
  }

  // for PRM
  int id;
  double g = costMax;
  double f = costMax;
};

bool isCollision(double* q1, double* q2, double* map, int x_size, int y_size, double dq, int numofDOFs)
{
  // q2 target, q1 start

  vector<double> direction;
  double cost = 0.0;
  for (size_t i = 0; i < numofDOFs; i++)
  {
	direction.push_back(q2[i] - q1[i]);
	double dist = q2[i] - q1[i];
	cost += dist * dist;
  }
  cost = sqrt(cost);

  int numofsamples = (int)(cost / dq);

  // from q_neighbor to q_new
  for (size_t i = 0; i < numofsamples; i++)
  {
	vector<double> tmp(numofDOFs, 0.0);
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  tmp[j] = q1[j] + ((double)(i + 1) / (numofsamples)) * (direction[j]);
	}
	if (!IsValidArmConfiguration(tmp.data(), numofDOFs, map, x_size, y_size))
	{
	  // not pass collistion check
	  return true;
	}
  }
  return false;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
			 int numofDOFs, double*** plan, int* planlength)
{
  // cout << "In planner function" << endl;

  // no plan by default
  *plan = NULL;
  *planlength = 0;

  // for now just do straight interpolation between start and goal checking for the validity of samples

  double distance = 0;
  int i, j;
  for (j = 0; j < numofDOFs; j++)
  {
	if (distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
	  distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
  }
  int numofsamples = (int)(distance / (PI / 20));
  if (numofsamples < 2)
  {
	printf("the arm is already at the goal\n");
	return;
  }
  *plan = (double**)malloc(numofsamples * sizeof(double*));
  int firstinvalidconf = 1;
  for (i = 0; i < numofsamples; i++)
  {
	(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
	for (j = 0; j < numofDOFs; j++)
	{
	  (*plan)[i][j] = armstart_anglesV_rad[j] +
					  ((double)(i) / (numofsamples - 1)) * (armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
	}
	if (!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
	{
	  firstinvalidconf = 1;
	  printf("ERROR: Invalid arm configuration!!!\n");
	}
  }
  *planlength = numofsamples;

  return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

vector<double> random_config(int numofDOFs, double* map, int x_size, int y_size, double* armgoal_anglesV_rad,
							 bool goalBias = false)
{
  // cout << "In random_config function" << endl;

  vector<double> result(numofDOFs);

  // goal bias
  if (goalBias && double(rand()) / double(RAND_MAX) > 0.90)
  {
	// cout << "goalBiased" << endl;
	double magnitude = 0.5;
	double randomPerturbation = magnitude * double(rand()) / double(RAND_MAX);
	for (size_t i = 0; i < numofDOFs; i++)
	{
	  result[i] = (armgoal_anglesV_rad[i] + (randomPerturbation - magnitude/2.0));
	}
	return result;
  }

  bool isValid = false;
  while (!isValid)
  {
	// cout << "isValid: " << isValid << endl;
	// use simple random number generator instead of std::mt19937
	// result.clear();
	for (size_t i = 0; i < numofDOFs; i++)
	{
	  double randomValue = 2 * PI * double(rand()) / double(RAND_MAX);
	  result[i] = randomValue;
	  // cout << randomValue << endl;
	}
	isValid = IsValidArmConfiguration(result.data(), numofDOFs, map, x_size, y_size);
  }
  return result;
};

vertex* nearest_neighbor(vector<vertex*>& tree, int& treeSize, int numofDOFs, vector<double> angles)
{
  // cout << "In nearest_neighbor function" << endl;

  // find the nearest neighbor
  vertex* nearest_neighbor = nullptr;
  double nearest_distance = 100000;

  // cout << "treeSize: " << treeSize << endl;
  // from back to front, find the nearest neighbor
  for (int i = treeSize - 1; i >= 0; i--)
  {
	double distance = 0;
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  // cout << "tree[i]->angles[j]: " << tree[i]->angles[j] << " ";
	  distance += (angles[j] - tree[i]->angles[j]) * (angles[j] - tree[i]->angles[j]);
	}
	distance = sqrt(distance);
	// cout << "distance: " << distance << endl;
	if (distance < nearest_distance)
	{
	  nearest_distance = distance;
	  nearest_neighbor = tree[i];
	}
  }
  return nearest_neighbor;
}

int extendRRT(vector<vertex*>& tree, int& treeSize, double dq, double epsilon, double* armgoal_anglesV_rad,
			  int numofDOFs, double* map, int x_size, int y_size, vector<double> target_angles,
			  bool isRRTConnect = false)
{
  // cout << "In extendRRT function" << endl;

  vertex* q_near = nearest_neighbor(tree, treeSize, numofDOFs, target_angles);

  // cout << "q_near: ";
  // for (size_t j = 0; j < numofDOFs; j++)
  // {
	// cout << q_near->angles[j] << " ";
  // }
  // cout << endl;

  vector<double> direction;
  double dist = 0;
  for (size_t i = 0; i < numofDOFs; i++)
  {
	double diff = target_angles[i] - q_near->angles[i];
	direction.push_back(diff);
	dist += diff * diff;
  }

  // normalize the direction
  dist = sqrt(dist);
  for (size_t i = 0; i < numofDOFs; i++)
  {
	direction[i] = direction[i] / dist;
	// cout << direction[i] << " ";
  }

  // cout << "target_angles: ";
  // for (size_t j = 0; j < numofDOFs; j++)
  // {
	// cout << target_angles[j] << " ";
  // }
  // cout << endl;

  int numofSamples = (int)(epsilon / dq);
  // cout << "numofSamples: " << numofSamples << endl;
  vector<double> new_angles(numofDOFs);
  for (size_t i = 1; i <= numofSamples; i++)
  {
	// cout << "new_angles: ";
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  new_angles[j] = (q_near->angles[j] + dq * i * direction[j]);
	  // cout << new_angles[j] << " ";
	}
	// cout << endl;
	if (IsValidArmConfiguration(new_angles.data(), numofDOFs, map, x_size, y_size))
	{
	  // If didn't reach the target or goal, go to next configuration
	  if (!isRRTConnect && roughlyEqualDoubleArrays(new_angles.data(), armgoal_anglesV_rad, numofDOFs))
	  {
		// Reach q_goal
		// GOAL_REACHED
		// add goal
		// cout << "GOAL_REACHED" << endl;
		vertex* q_new =
			new vertex(vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs), numofDOFs, q_near);
		q_new->g = q_near->g + vertex::calculateCost(q_near, q_new);
		tree.push_back(q_new);
		treeSize++;
		return 3;
	  }
	  if (roughlyEqualDoubleArrays(new_angles.data(), target_angles.data(), numofDOFs))
	  {
		// Reach target_angles, will enter this condition when dist < epsilon
		// REACHED
		// cout << "REACHED" << endl;
		vertex* q_new = new vertex(target_angles, numofDOFs, q_near);
		q_new->g = q_near->g + vertex::calculateCost(q_near, q_new);
		tree.push_back(q_new);
		treeSize++;
		return 2;
	  }
	}
	else
	{
	  if (i == 1)
	  {
		// TRAPPED
		// cout << "TRAPPED" << endl;
		return 0;
	  }
	  else
	  {
		// ADVANCED
		// cout << "ADVANCED" << endl;
		// take one step back
		for (size_t j = 0; j < numofDOFs; j++)
		{
		  new_angles[j] = new_angles[j] - dq * direction[j];
		  // cout << new_angles[j] << " ";
		}
		vertex* q_new = new vertex(new_angles, numofDOFs, q_near);
		q_new->g = q_near->g + vertex::calculateCost(q_near, q_new);
		tree.push_back(q_new);
		treeSize++;
		return 1;
	  }
	}
  }
  // Fully extended
  // ADVANCED
  // cout << "FULLY ADVANCED" << endl;
  vertex* q_new = new vertex(new_angles, numofDOFs, q_near);
  q_new->g = q_near->g + vertex::calculateCost(q_near, q_new);
  tree.push_back(q_new);
  treeSize++;
  return 1;
}

vector<vertex*> nearestKneighbors(vector<vertex*> tree, int treeSize, int numofDOFs, int k, vector<double> angles,
								  double* map, int x_size, int y_size, double dq)
{
  // cout << "In nearestKneighbors function" << endl;

  vector<vertex*> result;
  std::priority_queue<std::pair<double, vertex*>, std::vector<std::pair<double, vertex*>>,
					  std::greater<std::pair<double, vertex*>>> pq;

  // find the neighbors

  // cout << "treeSize: " << treeSize << endl;
  // from back to front, find the nearest k neighbor
  for (size_t i = treeSize - 1; i > 0; i--)
  {
	double distance = 0;
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  // cout << "tree[i]->angles[j]: " << tree[i]->angles[j] << " ";
	  distance += (angles[j] - tree[i]->angles[j]) * (angles[j] - tree[i]->angles[j]);
	}
	distance = sqrt(distance);
	pq.push({ distance, tree[i] });
  }
  // exclude self
  k++;
  while (!pq.empty() && k > 0)
  {
	if (!isCollision(pq.top().second->angles.data(), angles.data(), map, x_size, y_size, dq, numofDOFs))
	{
	  result.push_back(pq.top().second);
	  k--;
	}
	pq.pop();
  }
  return result;
};


// reference: Jinkai Qiu
vector<vector<double>> pathShortening(vector<vector<double>>& plan, int numofDOFs, double* map, int x_size, int y_size, double dq) 
{
	vector<vector<double>> shorten;
	shorten.push_back(plan[0]);
	int i = 1;
	vector<double> P = plan[0];
	vector<double> P1 = plan[1];
	vector<double> P2 = plan[i];
	vector<double> goal = plan[plan.size()-1];
	while (P != goal) {
		while (!isCollision(P.data(), P2.data(), map, x_size, y_size, dq, numofDOFs) && P2 != goal) {
			P1 = P2;
			i++;
			P2 = plan[i];
		}
		shorten.push_back(P1);
		P = P1;
		P1 = P2;
		}
	return shorten;
}


void makePlan(vector<vertex*>& tree, int treeSize, int numofDOFs, double* armgoal_anglesV_rad, double*** plan,
			  int* planlength, double* map, int x_size, int y_size, double dq)
{
  // backtracking to get the plan
  vector<vector<double>> plan_angles;

  vertex* q_curr = tree[treeSize - 1];
  while (q_curr)
  {
	plan_angles.push_back(q_curr->angles);
	q_curr = q_curr->parent;
	// (*planlength)++;
  }
	std::reverse(plan_angles.begin(), plan_angles.end());

  vector<vector<double>> shorten_plan_angles = pathShortening(plan_angles, numofDOFs, map, x_size, y_size, dq); 

  *planlength = shorten_plan_angles.size();

  *plan = (double**)malloc((*planlength) * sizeof(double*));

  for (size_t i = 0; i < *planlength; i++)
  {
	(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
	vector<double> angles = shorten_plan_angles[i];
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  (*plan)[i][j] = angles[j];
	}
  }
}


static void plannerRRT(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
					   int numofDOFs, double*** plan, int* planlength)
{
  /* TODO: Replace with your implementation */
  // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

  // cout << "In plannerRRT function" << endl;

  // no plan by default
  *plan = NULL;
  *planlength = 0;

  // extend parameter
  double dq = 0.01;
  double epsilon = 0.3;
  // max sample size
  // INT32_MAX = 2147483647
  int K = 20000000;

  // initialize the tree
  vector<vertex*> tree;
  int treeSize = 0;

  vertex* q_init =
	  new vertex(vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs), numofDOFs, nullptr);
  tree.push_back(q_init);
  treeSize++;

  bool goalReached = false;
  while (treeSize < K)
  {
	vector<double> rand_angles = random_config(numofDOFs, map, x_size, y_size, armgoal_anglesV_rad, true);
	if (extendRRT(tree, treeSize, dq, epsilon, armgoal_anglesV_rad, numofDOFs, map, x_size, y_size, rand_angles) == 3)
	{
	  goalReached = true;
	  break;
	}
  }

  if (!goalReached)
  {
	printf("ERROR: Plan not found.\n");
	return;
  }

  // construct the plan
  makePlan(tree, treeSize, numofDOFs, armgoal_anglesV_rad, plan, planlength, map, x_size, y_size, dq);

  cout << "Plan found." << endl;
  cout << "Vnum: " << treeSize << endl;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTConnect(double* map, int x_size, int y_size, double* armstart_anglesV_rad,
							  double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength)
{
  /* TODO: Replace with your implementation */
  // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

  // cout << "In plannerRRTConnect function" << endl;

  // no plan by default
  *plan = NULL;
  *planlength = 0;

  // extend parameter
  double dq = 0.01;
  double epsilon = 0.3;
  // max sample size
  // INT32_MAX = 2147483647
  int K = 20000000;

  // initialize the tree
  vector<vertex*> treeA;
  int treeASize = 0;
  vector<vertex*> treeB;
  int treeBSize = 0;

  vertex* q_init =
	  new vertex(vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs), numofDOFs, nullptr);
  treeA.push_back(q_init);
  treeASize++;

  vertex* q_goal = new vertex(vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs), numofDOFs, nullptr);
  treeB.push_back(q_goal);
  treeBSize++;

  // int iter = 0;
  bool connected = false;

  vector<vertex*>* forwardTree = &treeA;
  vector<vertex*>* backwareTree = &treeB;
  int* forwardTreeSize = &treeASize;
  int* backwareTreeSize = &treeBSize;

  while ((treeASize + treeBSize) < K)
  {
	vector<double> rand_angles = random_config(numofDOFs, map, x_size, y_size, armgoal_anglesV_rad, false);

	// try to extend treeA to q_rand
	int result = extendRRT(*forwardTree, *forwardTreeSize, dq, epsilon, armgoal_anglesV_rad, numofDOFs, map, x_size,
						   y_size, rand_angles, true);

	// 0: TRAPPED
	// 1: ADVANCED
	// 2: REACHED
	// 3: GOAL_REACHED
	if (result != 0)
	{
	  vector<double> targetAngles = (*forwardTree)[(*forwardTreeSize - 1)]->angles;
	  int outcome = 1;
	  do
	  {
		outcome = extendRRT(*backwareTree, *backwareTreeSize, dq, epsilon, armgoal_anglesV_rad, numofDOFs, map, x_size,
							y_size, targetAngles, true);
	  } while (outcome == 1);

	  if (outcome == 2)
	  {
		// Reach q_target so the two trees are connected
		connected = true;
		break;
	  }
	}

	// swap treeA and treeB
	auto tmpTree = forwardTree;
	forwardTree = backwareTree;
	backwareTree = tmpTree;
	auto tmpSize = forwardTreeSize;
	forwardTreeSize = backwareTreeSize;
	backwareTreeSize = tmpSize;
  }

  if (!connected)
  {
	printf("ERROR: Plan not found.\n");
	return;
  }

  // printf("Build plan\n");

  vector<vector<double>> plan_angles;
  // mid to goal
  vertex* q_curr = treeB[treeBSize - 1];
  while (q_curr)
  {
	plan_angles.push_back(q_curr->angles);
	q_curr = q_curr->parent;
  }
  // reverse so goal to mid now
  std::reverse(plan_angles.begin(), plan_angles.end());
  q_curr = treeA[treeASize - 1];
  // mid to start
  while (q_curr)
  {
	plan_angles.push_back(q_curr->angles);
	q_curr = q_curr->parent;
  }
  // again so start to goal now
  std::reverse(plan_angles.begin(), plan_angles.end());

  plan_angles = pathShortening(plan_angles, numofDOFs, map, x_size, y_size, dq);

  *planlength = plan_angles.size();
  *plan = (double**)malloc((*planlength) * sizeof(double*));

  for (size_t i = 0; i < *planlength; i++)
  {
	(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
	vector<double> angles = plan_angles[i];
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  (*plan)[i][j] = angles[j];
	}
  }

  cout << "Plan found." << endl;
  cout << "Vnum: " << treeASize + treeBSize << endl;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

vector<vertex*> neighbors(vector<vertex*> tree, int treeSize, int numofDOFs, double r, vector<double> angles, double* map,
						  int x_size, int y_size, double dq)
{
  // cout << "In neighbors function" << endl;

  vector<vertex*> result;
  std::priority_queue<std::pair<double, vertex*>, std::vector<std::pair<double, vertex*>>,
					  std::greater<std::pair<double, vertex*>>> pq;

  // find the neighbors

  // cout << "treeSize: " << treeSize << endl;
  // from back to front, find the nearest k neighbor
  for (size_t i = treeSize - 1; i > 0; i--)
  {
	double distance = 0;
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  // cout << "tree[i]->angles[j]: " << tree[i]->angles[j] << " ";
	  distance += (angles[j] - tree[i]->angles[j]) * (angles[j] - tree[i]->angles[j]);
	}
	distance = sqrt(distance);
	if (distance < r)
		pq.push({ distance, tree[i] });
  }
  
  // exclude self
  while (!pq.empty())
  {
	if (!isCollision(pq.top().second->angles.data(), angles.data(), map, x_size, y_size, dq, numofDOFs))
	{
	  result.push_back(pq.top().second);
	}
	pq.pop();
  }
  return result;
};

void rewire(std::vector<vertex*>& tree, int& treeSize, double dq, double epsilon, int numofDOFs, double* map,
			int x_size, int y_size)
{
  vertex* lastVertex = tree[treeSize - 1];
  lastVertex->id = treeSize;
  // cout << "lastVertex->id: " << lastVertex->id << endl;

  double gamma = 3.7;
  // volume of unit ball in n-dimension
  double delta = pow(PI, numofDOFs/2.0)/tgamma(numofDOFs/2.0 + 1);
  double r_candidate = pow((gamma/delta * log(treeSize)/treeSize),1.0/numofDOFs);
  double r = std::min(r_candidate, epsilon);
  // cout << "r_candidate: " << r_candidate << ", r: " << r << ", epsilon: " << epsilon << endl;
  vector<vertex*> nbs = neighbors(tree, treeSize, numofDOFs, r, lastVertex->angles, map, x_size, y_size, dq);

  for (auto neighbor : nbs)
  {
	// cout << "neighbor->id: " << neighbor->id << endl;
	double newCost = neighbor->g + vertex::calculateCost(lastVertex, neighbor);
	if (newCost < lastVertex->g &&
		!isCollision(neighbor->angles.data(), lastVertex->angles.data(), map, x_size, y_size, dq, numofDOFs))
	{
	  // cout << "rewire lastVertex" << endl;
	  lastVertex->parent = neighbor;
	  lastVertex->g = newCost;
	}
  }

  for (auto neighbor : nbs)
  {
	double newCost = lastVertex->g + vertex::calculateCost(neighbor, lastVertex);
	if (newCost < neighbor->g &&
		!isCollision(neighbor->angles.data(), lastVertex->angles.data(), map, x_size, y_size, dq, numofDOFs))
	{
	  // cout << "rewire neighbor" << endl;
	  neighbor->parent = lastVertex;
	  neighbor->g = newCost;
	}
  }
  return;
}

int extendRewire(vector<vertex*>& tree, int& treeSize, double dq, double epsilon, double* armgoal_anglesV_rad,
				 int numofDOFs, double* map, int x_size, int y_size, vector<double> angles)
{
  int result = extendRRT(tree, treeSize, dq, epsilon, armgoal_anglesV_rad, numofDOFs, map, x_size, y_size, angles);
  rewire(tree, treeSize, dq, epsilon, numofDOFs, map, x_size, y_size);
  return result;
}

static void plannerRRTStar(double* map, int x_size, int y_size, double* armstart_anglesV_rad,
						   double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength)
{
  /* TODO: Replace with your implementation */
  // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

  // cout << "In plannerRRTStar function" << endl;

  // no plan by default
  *plan = NULL;
  *planlength = 0;

  // extend parameter
  double dq = 0.01;
  double epsilon = 0.3;
  // max sample size
  // INT32_MAX = 2147483647
  int K = 20000000;

  // initialize the tree
  vector<vertex*> tree;
  int treeSize = 0;

  vertex* q_init =
	  new vertex(vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs), numofDOFs, nullptr);
  q_init->g = 0;
  q_init->id = 1;
  tree.push_back(q_init);
  treeSize++;

  bool goalReached = false;
  while (treeSize < K)
  {
	vector<double> rand_angles = random_config(numofDOFs, map, x_size, y_size, armgoal_anglesV_rad, true);
	if (extendRewire(tree, treeSize, dq, epsilon, armgoal_anglesV_rad, numofDOFs, map, x_size, y_size, rand_angles) == 3)
	{
	  goalReached = true;
	  break;
	}
  }

  if (!goalReached)
  {
	printf("ERROR: Plan not found.\n");
	return;
  }

  // construct the plan
  makePlan(tree, treeSize, numofDOFs, armgoal_anglesV_rad, plan, planlength, map, x_size, y_size, dq);

  cout << "Plan found." << endl;
  cout << "Vnum: " << treeSize << endl;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

struct CompareVertex
{
  bool operator()(vertex* n1, vertex* n2)
  {
	// more false, more priority
	return n1->f > n2->f;
  }
};

vector<vertex*> astar(int numofDOFs, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
					 std::vector<vertex*>& graph, std::vector<std::unordered_set<int>>& adjacency)
{
  vector<vertex*> path;
  bool foundPath = false;
  std::priority_queue<vertex*, vector<vertex*>, CompareVertex> open_pq;
  std::unordered_set<vertex*> closed;
  open_pq.push(graph[0]);
  vertex* cur = nullptr;
  while (!open_pq.empty())
  {
	cur = open_pq.top();
	open_pq.pop();
	closed.insert(cur);
	// cout << "cur->id: " << cur->id << endl;
	if (cur == graph[1])
	{
	  foundPath = true;
	  break;
	}
	for (int neighborId : adjacency[cur->id])
	{
	  vertex* neighbor = graph[neighborId];
	  if (closed.find(neighbor) == closed.end())
	  {
		// not closed
		double h = vertex::calculateCost(neighbor, graph[1]);
		double cost = vertex::calculateCost(cur, neighbor);
		if (cur->g + cost < neighbor->g)
		{
		  neighbor->g = cur->g + cost;
		  neighbor->f = neighbor->g + h;
		  neighbor->parent = cur;
		  open_pq.push(neighbor);
		}
	  }
	}
  }
  if (!foundPath)
  {
	return vector<vertex*>();
  }
  while (cur != nullptr)
  {
	path.push_back(cur);
	cur = cur->parent;
  }
  return path;
}

static void plannerPRM(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
					   int numofDOFs, double*** plan, int* planlength)
{
  /* TODO: Replace with your implementation */
  // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

  // cout << "In plannerPRM function" << endl;

  // no plan by default
  *plan = NULL;
  *planlength = 0;

  // max sample size
  // INT32_MAX = 2147483647

  // collision checking
  double dq = 0.01;

  // initialize the graph
  vector<vertex*> graph;
  vector<std::unordered_set<int>> adjacency;
  int graphSize = 0;
  int edgeSize = 0;

  vertex* q_init =
	  new vertex(vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs), numofDOFs, nullptr);
  q_init->g = 0;
  q_init->id = 0;
  vertex* q_goal = new vertex(vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs), numofDOFs, nullptr);
  // should the g be max?
  q_goal->id = 1;
  graph.push_back(q_init);
  graph.push_back(q_goal);
  graphSize += 2;
  adjacency.push_back(std::unordered_set<int>());
  adjacency.push_back(std::unordered_set<int>());
  edgeSize += 2;

  // sample K configurations
  int k = 2;

  int maxIter = 10;
  int near_k = 5;
  int iter_k = 5000;
  bool planFound = false;
  vector<vertex*> path;
  for (size_t iter = 1; iter <= maxIter; iter++)
  {
    while (k < iter_k * iter)
    {
    vector<double> rand_angles = random_config(numofDOFs, map, x_size, y_size, armgoal_anglesV_rad, false);
    // cout << "rand_angles: ";
    // for (size_t i = 0; i < numofDOFs; i++)
    // {
      // cout << rand_angles[i] << " ";
    // }
    // cout << endl;
    vector<double> near_rand_angles(numofDOFs);

    // cout << "near_rand_angles: ";
    for (size_t i = 0; i < numofDOFs; i++)
    {
      double magnitude = 1.0;
      double randomPerturbation = magnitude * double(rand()) / double(RAND_MAX);
      near_rand_angles[i] = (rand_angles[i] + (randomPerturbation - magnitude / 2));
      // cout << near_rand_angles[i] << " ";
    }
    // cout << endl;

    if (!IsValidArmConfiguration(near_rand_angles.data(), numofDOFs, map, x_size, y_size))
    {
      vertex* q_new = new vertex(rand_angles, numofDOFs, nullptr);
      q_new->id = k;
      graph.push_back(q_new);
      graphSize++;
      adjacency.push_back(std::unordered_set<int>());
      edgeSize++;
      k++;
    }
    }

    // connect the graph
    for (vertex* v : graph)
    {
    // cout << "v->id: " << v->id << endl;
    vector<vertex*> neighborhood =
      nearestKneighbors(graph, graphSize, numofDOFs, near_k*iter, v->angles, map, x_size, y_size, dq);
    for (vertex* neighbor : neighborhood)
    {
      // cout << "neighbor->id: " << neighbor->id << endl;
      // exclude self
      if (v->id != neighbor->id)
      {
      adjacency[v->id].insert(neighbor->id);
      adjacency[neighbor->id].insert(v->id);
      }
    }
    }

    // find the shortest path
    path = astar(numofDOFs, armstart_anglesV_rad, armgoal_anglesV_rad, graph, adjacency);
    if (!path.empty())
    {
      planFound = true;
      break;
    }

  }

  // cout << "path.size(): " << path.size() << endl;

  if (!planFound)
  {
	printf("ERROR: Plan not found.\n");
	return;
  }

  *planlength = path.size();
  *plan = (double**)malloc((*planlength) * sizeof(double*));

  std::reverse(path.begin(), path.end());

  for (size_t i = 0; i < *planlength; i++)
  {
	(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
	// cout << "path.top()->id: " << path.top()->id << endl;
	vector<double> angles = path[i]->angles;
	for (size_t j = 0; j < numofDOFs; j++)
	{
	  (*plan)[i][j] = angles[j];
	}
  }

  cout << "Plan found." << endl;
  cout << "Vnum: " << graphSize << endl;
  return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos,
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv)
{
  double* map;
  int x_size, y_size;

  std::string mapDirPath = MAPS_DIR;
  std::string mapFilePath = mapDirPath + "/" + argv[1];
  std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
  tie(map, x_size, y_size) = loadMap(mapFilePath);
  const int numOfDOFs = std::stoi(argv[2]);
  double* startPos = doubleArrayFromString(argv[3]);
  double* goalPos = doubleArrayFromString(argv[4]);
  int whichPlanner = std::stoi(argv[5]);

  std::string outputDir = OUTPUT_DIR;
  string outputFile = outputDir + "/" + argv[6];
  std::cout << "Writing solution to: " << outputFile << std::endl;

  if (!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) ||
	  !IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size))
  {
	throw runtime_error("Invalid start or goal configuration!\n");
  }

  ///////////////////////////////////////
  //// Feel free to modify anything below. Be careful modifying anything above.

  double** plan = NULL;
  int planlength = 0;

  srand(1);

  // Call the corresponding planner function
  // RRT: 0, RRTConnect: 1, RRT*: 2, PRM: 3
  if (whichPlanner == PRM)
  {
	plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else if (whichPlanner == RRT)
  {
	plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else if (whichPlanner == RRTCONNECT)
  {
	plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else if (whichPlanner == RRTSTAR)
  {
	plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else
  {
	planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }

  //// Feel free to modify anything above.
  //// If you modify something below, please change it back afterwards as my
  //// grading script will not work and you will recieve a 0.
  ///////////////////////////////////////

  // Your solution's path should start with startPos and end with goalPos
  if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || !equalDoubleArrays(plan[planlength - 1], goalPos, numOfDOFs))
  {
	throw std::runtime_error("Start or goal position not matching");
  }

  /** Saves the solution to output file
   * Do not modify the output log file output format as it is required for visualization
   * and for grading.
   */
  std::ofstream m_log_fstream;
  m_log_fstream.open(outputFile, std::ios::trunc);	// Creates new or replaces existing file
  if (!m_log_fstream.is_open())
  {
	throw std::runtime_error("Cannot open file");
  }
  m_log_fstream << mapFilePath << endl;	 // Write out map name first
  /// Then write out all the joint angles in the plan sequentially
  for (int i = 0; i < planlength; ++i)
  {
	for (int k = 0; k < numOfDOFs; ++k)
	{
	  m_log_fstream << plan[i][k] << ",";
	}
	m_log_fstream << endl;
  }
}
