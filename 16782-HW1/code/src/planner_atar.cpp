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

map<double, Node*> open_pq;
unordered_set<string> closed_set;
vector<Node*> path;
// starts with 1 to skip the start state
int pathIndex = 1;
int pathSize = 0;
bool foundPath = false;

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

    if (!foundPath) {


        // int goalposeX = target_traj[curr_time];
        // int goalposeY = target_traj[curr_time+target_steps];

        int goalposeX = target_traj[target_steps - 1];
        int goalposeY = target_traj[target_steps - 1 + target_steps];
        Node* goal_state = new Node(goalposeX, goalposeY, INFINITE_COST, 0, nullptr);
        cout << "goal state: " << endl;
        Node::printNode(*goal_state);

        // initialize with start state
        if (curr_time == 0) {
            Node* start_state = new Node(robotposeX, robotposeY, 0, 0, nullptr);
            start_state->update(goalposeX, goalposeY);
            cout << "start state: " << endl;
            Node::printNode(*start_state);
            open_pq[start_state->f] = start_state;

            // put entire goal traj into a set for lookup
            // for (int i = 0; i < target_steps; i ++) {
            //     Node* goal_state = new Node(target_traj[i], target_traj[i+target_steps], i, __INT_MAX__, 0);
            // }

        }

        // A* search
        while (open_pq.size() > 0) {

            Node* topPriorityNode = open_pq.begin()->second;
            open_pq.erase(open_pq.begin());
            cout << "topPriorityNode: " << endl;
            Node::printNode(*topPriorityNode);
            closed_set.insert(topPriorityNode->hash());

            if (topPriorityNode->hash() == goal_state->hash()) {
                goal_state->parent = topPriorityNode->parent;
                foundPath = true;
                break;
            }

            for(int dir = 0; dir < NUMOFDIRS; dir++) {

                int newx = topPriorityNode->x + dX[dir];
                int newy = topPriorityNode->y + dY[dir];

                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {

                    int new_cost = map[GETMAPINDEX(newx,newy,x_size,y_size)];

                    // if free
                    if ((new_cost >= 0) && (new_cost < collision_thresh)) {
                        cout << "free" << endl;

                        Node* new_state = new Node(newx, newy, INFINITE_COST, 0, topPriorityNode);
                        new_state->update(goalposeX, goalposeY);
                        cout << "new state: " << endl;
                        Node::printNode(*new_state);

                        // if not closed
                        if (closed_set.find(new_state->hash()) == closed_set.end()) {
                            cout << "not closed" << endl;
                            if (new_state->g > topPriorityNode->g + 1)
                                new_state->g = topPriorityNode->g + 1;

                                bool foundNode = false;
                                for (auto it = open_pq.begin(); it != open_pq.end(); ++it) {
                                    if (it->second->hash() == new_state->hash()) {
                                        if (new_state->g < it->second->g) {
                                            cout << "found in open, update" << endl;
                                            it->second->g = new_state->g;
                                            it->second->update(goalposeX, goalposeY);
                                            // update parent
                                            it->second->parent = topPriorityNode;
                                        }
                                        break;
                                    }
                                }
                                if (!foundNode) {
                                    cout << "not found in open either, push to open" << endl;
                                    open_pq[new_state->f] = new_state;
                                }                       

                        } else {
                            cout << "closed" << endl;
                        }
                    } else {
                        cout << "obstacle or out of boundary" << endl;
                    }
                }
            }

        } 

        // backtrace
        Node* curr = goal_state;
        while (curr != nullptr) {
            path.push_back(curr);
            curr = curr->parent;
        }
        reverse(path.begin(), path.end());
        pathSize = path.size();
        cout << "path size: " << pathSize << endl;

        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;

    } else {
        
        // cout << robotposeX << " " << robotposeY << endl;
        // cout << path[path_i]->x << " " << path[path_i]->y << endl;
        if (pathIndex != pathSize) {
            action_ptr[0] = path[pathIndex]->x;
            action_ptr[1] = path[pathIndex]->y;
            pathIndex++;
        }


    }

    return;
}