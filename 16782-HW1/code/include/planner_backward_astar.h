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
        this->f = g + h;
    }

    void update() {

        // f = g + epsilon * h;
        f = g + h;
    }

    void updateSingle(int target_x, int target_y) {

        h = heuristic(target_x, target_y);
        // f = g + epsilon * h;
        f = g + h;
    }

    double heuristic(int target_x, int target_y) {
        // use Euclidean dist as heuristic val
        // turn time also into dist heuristic?
        return sqrt((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));
    }

    void updateMulti(int target_steps, int* target_traj) {

        for (int i = 0; i < target_steps; i ++) {
            h = min(h, heuristic(target_traj[i], target_traj[i+target_steps]));
        }
        // f = g + epsilon * h;
        f = g + h;
    }

    string hash() {
        return to_string(x) + "ganninian" + to_string(y);
    }

    static void printNode(Node n) {
        // cout << "x: " << n.x << ", y: " << n.y << ", t: " << n.t << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
        cout << "x: " << n.x << ", y: " << n.y << ", g: " << n.g << ", h: " << n.h << ", f: " << n.f << endl;
    }

};

#endif // PLANNER_H