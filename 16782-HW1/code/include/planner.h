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

    // const double epsilon = 1.0;

    int x;
    int y;
    int t;
    int g;
    double h;
    double f;
    bool operator<(const Node& other) const {
        // reversed comparison for min-heap
        return this->f > other.f;
    }
    Node* parent;

    Node(int x, int y, int g, int h, Node* parent) {
        this->x = x;
        this->y = y;
        this->g = g;
        this->h = h;
        // t = 0;
        this->parent = parent;
        // this->f = g + epsilon * h;
        update();
    }

    void update() {

        f = g + h;
    }
    
    double heuristic(int target_x, int target_y) {

        // use Euclidean dist as heuristic val
        return sqrt((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));
        
    }

    void updateMulti(int target_steps, int* target_traj) {

        // find the min heuristic value among all target states

        // double tmp = INFINITE_COST;

        // for (int i = 0; i < target_steps; i ++) {
        //     tmp = min(tmp, heuristic(target_traj[i], target_traj[i+target_steps]));
        // }

        double tmp = 0.0;
        int ratio = 10;
        for (int i = 0; i < target_steps; i += target_steps/ratio) {
            tmp += heuristic(target_traj[i], target_traj[i+target_steps]);
        }

        // double tmp = heuristic(target_traj[target_steps/2], target_traj[target_steps/2 + target_steps]);

        h = tmp;
        update();
    }

    string hash() {
        return to_string(x) + ")(" + to_string(y);
    }

    static void printNode(Node n) {
        // cout << "x: " << n.x << ", y: " << n.y << ", t: " << n.t << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
        cout << "x: " << n.x << ", y: " << n.y << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
    }

};

#endif // PLANNER_H