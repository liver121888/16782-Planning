/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// 8-connected grid
int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

map<double, string> open_pq;

unordered_set<string> closed_set;
// start_set is storing all the target states
unordered_map<string, int> start_set;
// g, t, hash
map<int, pair<int, string>> candidate_set;

unordered_map<string, Node*> grid;


Node* curr;
vector<Node*> path;
// starts with 1 to skip the start state
int pathIndex = 1;
int pathSize = 0;
int plannerState = 0;

//TODO: update the parent of the goal state
void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
) { 


    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    return;
}