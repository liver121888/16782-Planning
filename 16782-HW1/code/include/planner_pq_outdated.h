#ifndef PLANNER_H
#define PLANNER_H

// __INT_MAX__ = 2147483647
#define INFINITE_COST 2000000000

#include <math.h>
#include <queue>
#include <vector>
#include <iostream>
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
    const double epsilon = 1.0;

    int version;
    int x;
    int y;
    int t;
    int g;
    double h;
    double f;

    string parentHash;
    string leastCostParentHash;

    Node(int x, int y, int t, int g, int h, int version, string parent, string leastCostParent) {
        this->x = x;
        this->y = y;
        this->g = g;
        this->h = h;
        this->t = t;
        this->version = version;
        this->parentHash = parent;
        this->leastCostParentHash = leastCostParent;
        update();
        // this->f = g + h;
    }

    void update() {

        f = g + epsilon * h;
    }

    double heuristic(int target_x, int target_y) {
        // use Euclidean dist as heuristic val
        // turn time also into dist heuristic?
        return sqrt((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));
    }

    void updateMulti(int target_steps, int* target_traj) {

        // find the min heuristic value among all target states

        double tmp = INFINITE_COST;

        // double tmp = 0.0;
        // // the higher the finer the resolution 
        int resolution = target_steps/100;


        for (int i = 0; i < target_steps; i += target_steps/resolution) {
            tmp = min(tmp, heuristic(target_traj[i], target_traj[i+target_steps]));
        }

        // for (int i = 0; i < target_steps; i += target_steps/resolution) {
        //     tmp += heuristic(target_traj[i], target_traj[i+target_steps]);
        // }
        // tmp /= target_steps/resolution;
        // double tmp = heuristic(target_traj[target_steps/2], target_traj[target_steps/2 + target_steps]);

        h = tmp;
        update();
    }

    string hash() {
        return to_string(x) + " " + to_string(y);
    }

    static void printNode(Node n) {
        cout << "x: " << n.x << ", y: " << n.y << ", t: " << n.t << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
        // cout << "x: " << n.x << ", y: " << n.y << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
    }

};

#endif // PLANNER_H