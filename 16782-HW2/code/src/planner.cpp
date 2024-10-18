/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <stack>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

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

// the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


// some potentially helpful imports
using std::vector;
using std::stack;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            // cout << endl;
            return false;
        }
    }
    return true;
}

bool roughlyEqualDoubleArrays(double* v1, double *v2, int size) {
	double thresh = 1e-1;
	double curSum = 0;
    for (int i = 0; i < size; ++i) {
		curSum += (v1[i]-v2[i]) * (v1[i]-v2[i]);
	}
	curSum = sqrt(curSum);
	if (curSum < thresh) {
		return true;
	}
	return false;
}

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

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	// take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
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

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	// printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	// make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	// printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	// iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	// iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		// compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		// check the validity of the corresponding line segment
		// line segment means a link
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(
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
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

struct vertex
{
	/* data */
	vector<double> angles;
	int numofDOFs;
	vertex* parent;

	// Constructor to initialize the vertex
    vertex(vector<double> angles, int numofDOFs, vertex* parent = nullptr)
        : angles(angles), numofDOFs(numofDOFs), parent(parent)
    {

    }
};

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	cout << "In plannerRRT function" << endl;

	// set seed to 1
	srand(1);

	// no plan by default
	*plan = NULL;
	*planlength = 0;

	// extend parameter
	double epsilon = 0.1;
	// sample size
	// INT32_MAX = 2147483647
	int K = 20000000;

	// initialize the tree
	vector<vertex*> tree;
	int treeSize = 0;

    auto random_config = [numofDOFs, armgoal_anglesV_rad]() -> vector<double> {
		
		// cout << "In random_config function" << endl;

		// goal bias
		if (double(rand())/double(RAND_MAX) > 0.95) {
			return vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
		}

		// use simple random number generator instead of std::mt19937
		vector<double> result;
		for (size_t i = 0; i < numofDOFs; i++)
		{
			double randomValue = 2*PI*double(rand())/double(RAND_MAX);
			result.push_back(randomValue);
			// cout << randomValue << endl;
		}
		return result;
    };

	auto nearest_neighbor = [&tree, &treeSize, numofDOFs](vector<double> angles) -> vertex* {
		
		// cout << "In nearest_neighbor function" << endl;

		// find the nearest neighbor
		vertex* nearest_neighbor = nullptr;
		double nearest_distance = 100000;

		// cout << "treeSize: " << treeSize << endl;

		for (size_t i = 0; i < treeSize; i++)
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
	};

	auto extend = [&tree, &treeSize, epsilon, armgoal_anglesV_rad, nearest_neighbor, numofDOFs, map, x_size, y_size](vector<double> angles) -> int {

		// cout << "In extend function" << endl;

		vertex* q_near = nearest_neighbor(angles);

		cout << "q_near: ";
		for (size_t i = 0; i < numofDOFs; i++)
		{
			cout << q_near->angles[i] << " ";
		}
		cout << "/ q_rand: ";
		for (size_t i = 0; i < numofDOFs; i++)
		{
			cout << angles[i] << " ";
		}
		cout << endl;

		// cout << q_near << endl;

		// calculate the new configuration
		vector<double> direction;
		double squared_sum = 0;
		for (size_t i = 0; i < numofDOFs; i++)
		{
			double dist = angles[i] - q_near->angles[i];
			direction.push_back(dist);
			squared_sum += dist * dist;
		}
		// normalize and extend epsilon the direction
		squared_sum = sqrt(squared_sum);
		for (size_t i = 0; i < numofDOFs; i++)
		{
			direction[i] = epsilon * direction[i] / squared_sum;
			// cout << direction[i] << " ";
		}
		int maxExtend = floor(squared_sum/epsilon);
		cout << maxExtend << endl;

		bool shouldPrint = false;
		if (abs(angles[0] - 0.392) < 1e-3)
		{
			cout << "maxExtend: " << maxExtend << endl;
			shouldPrint = true;
		}


		bool isValid = true;
		while (isValid)
		{
			vector<double> new_angles;
			// cout << "new_angles: ";
			for (size_t i = 0; i < numofDOFs; i++) {
				new_angles.push_back(q_near->angles[i] + direction[i]);
				// cout << new_angles[i] << " ";
			}
			// cout << endl;
			
			if (shouldPrint)
			{
				cout << "new_angles: ";
				for (size_t i = 0; i < numofDOFs; i++)
				{
					cout << new_angles[i] << " ";
				}
				cout << endl;
			}

			// check if the new configuration is valid
			isValid = IsValidArmConfiguration(new_angles.data(), numofDOFs, map, x_size, y_size);
			// cout << "isValid: " << isValid << endl;
			maxExtend--;
			if (isValid)
			{
				tree.push_back(new vertex(new_angles, numofDOFs, q_near));
				treeSize++;
				// cout << treeSize << endl;
				// update q_near
				q_near = tree[treeSize - 1];
				if (roughlyEqualDoubleArrays(new_angles.data(), armgoal_anglesV_rad, numofDOFs))
				{
					// Reach q_goal
					// cout << isValid << endl;
					cout << "Reach q_goal" << endl;
					return 1;
				}
				if (roughlyEqualDoubleArrays(new_angles.data(), angles.data(), numofDOFs))
				{
					// Reach q_rand
					// cout << isValid << endl;
					cout << "Reach q_rand" << endl;
					return 0;
				}
			}
		}
		return 0;
	};

	vertex* q_init = new vertex(vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs), numofDOFs, nullptr);
	tree.push_back(q_init);
	treeSize++;

	// int iter = 0;
	bool goalReached = false;
	while (treeSize < K) 
	{
		// iter++;
		// cout << "Iteration: " << iter << ", treeSize: " << treeSize << endl;
		vector<double> rand_angles = random_config();
		if (extend(rand_angles) == 1) {
			goalReached = true;
			break;
		}
	}

	if (!goalReached) {
		printf("ERROR: Plan not found.\n");
		return;
	}

	// backtracking to get the plan
	stack<vector<double>> plan_angles;

	// add goal
	plan_angles.push(vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs));
	(*planlength)++;
    
	vertex* q_curr = tree[treeSize - 1];
	while (q_curr)
	{
		plan_angles.push(q_curr->angles);
		q_curr = q_curr->parent;
		(*planlength)++;
	}

	*plan = (double**)malloc((*planlength) * sizeof(double*));
	
	for (size_t i = 0; i < *planlength; i++)
	{
		(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
		vector<double> angles = plan_angles.top();
		plan_angles.pop();
		for (size_t j = 0; j < numofDOFs; j++)
		{
			(*plan)[i][j] = angles[j];
		}
	}

	cout << "Plan found." << endl;
    return;	
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);





}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
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
int main(int argc, char** argv) {
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

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
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
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
