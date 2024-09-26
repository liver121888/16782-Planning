#ifndef PLANNER_H
#define PLANNER_H

// __INT_MAX__ = 2147483647
#define INFINITE_COST 20000000

#include <math.h>
#include <queue>
#include <vector>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <stack>

using namespace std;


// Declare the plan function
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
);



class Node {
public:

    // set to 1.0 to ensure optimality
    // const int epsilon = 10;

    int version;
    int x;
    int y;
    int t;
    int g;
    string hash;
    int h;
    int f;

    string parentHash;
    string leastCostParentHash;

    Node(int x, int y, int t, int g, int h, int version, string parent, string leastCostParent) {
        this->x = x;
        this->y = y;
        this->hash = to_string(x) + " " + to_string(y);
        this->g = g;
        this->h = h;
        this->t = t;
        this->version = version;
        this->parentHash = parent;
        this->leastCostParentHash = leastCostParent;
        update();

    }

    void update() {

        // f = g + epsilon * h;
        f = g + h;

    }

    // double heuristic(int target_x, int target_y, int target_t) {
    //     // use Euclidean dist as heuristic val
    //     // turn time also into dist heuristic?

    //     return sqrt((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));
    // }

    void updateMulti(int target_steps, int* target_traj) {

        if (t > target_steps) {
            // impossible to reach
            h = INFINITE_COST;
            update();
            return;
        }     

        // find the min heuristic value among all target states

        int tmp = INFINITE_COST;

        // double tmp = 0.0;
        // // the higher the finer the resolution 
        // int resolution = target_steps/target_steps;

        // Loop over future time steps from current time t to target_steps
        for (int i = t; i < target_steps; ++i) {

            int target_x = target_traj[i];
            int target_y = target_traj[i + target_steps];
        
            // Compute minimal steps (Chebyshev distance) to reach target's position at time i
            int time_needed = max(abs(x - target_x), abs(y - target_y));
                
            // Check if we can reach the target's position at time i
            if (time_needed <= i - t) {
                tmp = min(tmp, time_needed);
            }
        }

        // for (int i = 0; i < target_steps; i += target_steps/resolution) {

        //     // Compute minimal steps (Chebyshev distance) to reach target's position at time i
        //     int d = max(abs(x - target_x), abs(y - target_y));

        //     tmp = min(tmp, heuristic(target_traj[i], target_traj[i+target_steps], i));
        // }

        h = tmp;
        update();
    }

    // string hash() {
    //     return to_string(x) + " " + to_string(y);
    // }

    static void printNode(Node n) {
        cout << "x: " << n.x << ", y: " << n.y << ", t: " << n.t << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
        // cout << "x: " << n.x << ", y: " << n.y << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
    }

};

#endif // PLANNER_H