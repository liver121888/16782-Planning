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

// map<int, string> open_pq;

// Define a custom comparator
struct Compare {
    bool operator()(const std::pair<double, std::string>& a, const std::pair<double, std::string>& b) {
        return a.first > b.first; // For min-heap, use '>' instead of '<'
    }
};
priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, Compare> open_pq;

unordered_set<string> closed_set;
// start_set is storing all the target states
unordered_map<string, int> start_set;
map<int, string> candidate_set;

unordered_map<string, Node*> grid;


Node* curr;
vector<Node*> path;
// starts with 1 to skip the start state
int pathIndex = 1;
int pathSize = 0;
bool foundPath = false;

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

    if (!foundPath) {

        Node* start_state = new Node(robotposeX, robotposeY, 0, 0, nullptr);
        start_state->updateMulti(target_steps, target_traj);
        // cout << "start state (goal): " << endl;
        // Node::printNode(*start_state);
        open_pq.push(make_pair(start_state->f, start_state->hash()));
        grid[start_state->hash()] = start_state;
        // cout << start_state->hash() << endl;

        // Backward A* search
        // Put all the interested states into start_set
        // Assumption: target_traj never cross start state
        // vector<int>: in case target cross same position multiple times
        for (int i = 0; i < target_steps; i++) {
            Node* goal_state = new Node(target_traj[i], target_traj[i+target_steps], INFINITE_COST, 0, nullptr);
            // Node::printNode(*goal_state);
            start_set[goal_state->hash()] = i;
            // delete goal_state;
            // cout << i << ", start_hash: " << goal_state->hash() << endl;

        }

        cout << "===================" << endl;
        cout << "start_set.size(): " << start_set.size() << ", open_pq.size(): " << open_pq.size() << endl;

        while (start_set.size() > 0 && open_pq.size() > 0) {

            // for (auto it = open_pq.begin(); it != open_pq.end(); ++it) {
            //     cout << it->first << " " << it->second << endl;
            // }

            auto aPair = open_pq.top();
            open_pq.pop();
            Node* topPriorityNode = grid.at(aPair.second);
            // Node::printNode(*topPriorityNode);

            // open_pq.erase(open_pq.begin());
            // cout << "topPriorityNode: " << endl;
            // Node::printNode(*topPriorityNode);
            closed_set.insert(topPriorityNode->hash());

            cout << "check_hash: " << topPriorityNode->hash() << endl;

            if (start_set.find(topPriorityNode->hash()) != start_set.end()) {
                // cout << start_set[topPriorityNode->hash()] << " " << topPriorityNode->g << endl;
                // vector<int> times = start_set[topPriorityNode->hash()];
                // for (auto time : times) {
                if (start_set[topPriorityNode->hash()] >= topPriorityNode->g) {
                    cout << "goal reachable" << endl;
                    candidate_set[topPriorityNode->g] = topPriorityNode->hash();
                } else {
                    cout << "goal unreachable" << endl;
                }
                // }
                start_set.erase(topPriorityNode->hash());
                cout << "start_set.size(): " << start_set.size() << endl;
            }

            for(int dir = 0; dir < NUMOFDIRS; dir++) {

                int newx = topPriorityNode->x + dX[dir];
                int newy = topPriorityNode->y + dY[dir];

                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {

                    int new_cost = map[GETMAPINDEX(newx,newy,x_size,y_size)];

                    // if free
                    if ((new_cost >= 0) && (new_cost < collision_thresh)) {
                        // cout << "free" << endl;

                        // initialize g val with INFINITE_COST                      

                        Node* new_state = new Node(newx, newy, INFINITE_COST, 0, topPriorityNode);
                        new_state->updateMulti(target_steps, target_traj);


                        bool inGrid = false;

                        if (grid.find(new_state->hash()) != grid.end()) {
                            // cout << "found in grid" << endl;
                            inGrid = true;
                            new_state = grid.at(new_state->hash());
                        }

                        // if not closed
                        if (closed_set.find(new_state->hash()) == closed_set.end()) {
                            // cout << "not closed" << endl;

                            if (new_state->g > topPriorityNode->g + 1) {
                                new_state->g = topPriorityNode->g + 1;
                                cout << "gg: " << new_state->g << endl;
                                new_state->update();

                                bool foundNode = false;
                                // for (auto it = open_pq.begin(); it != open_pq.end(); ++it) {

                                //     if (new_state->hash() == it->second) {
                                //         foundNode = true;
                                //         // cout << "found in open" << endl;

                                //         open_pq.erase(it);
                                //         // update the priority queue
                                //         // cout << "update the priority queue" << endl;
                                //         open_pq[new_state->g] = new_state->hash();

                                //         grid[new_state->hash()] = new_state;
                                //         break;
                                //     }
                                // }

                                std::vector<std::pair<double, std::string>> temp;

                                // Remove elements from pq and modify the specific one
                                while (!open_pq.empty()) {
                                    auto element = open_pq.top();
                                    open_pq.pop();

                                    if (element.second == new_state->hash()) {
                                        // Update the double value
                                        element.first = new_state->f;
                                    }

                                    // Store in a temporary vector
                                    temp.push_back(element);
                                }

                                // Push everything back into the priority queue
                                for (const auto& element : temp) {
                                    open_pq.push(element);
                                }

                                if (!foundNode) {
                                    // cout << "not found in open either, push to open" << endl;

                                    // cout << "f: "<< new_state->f << endl;

                                    // insert into open
                                    open_pq.push(make_pair(new_state->f, new_state->hash()));
                                    grid[new_state->hash()] = new_state;
                                }

                            } else {
                                // cout << "not better" << endl;
                                
                                // open_pq[new_state->f] = new_state->hash();

                            }  

                        } else {
                            // cout << "closed" << endl;
                            delete new_state;
                        }
                    } else {
                        // cout << "obstacle" << endl;
                    }
                } else {
                    // cout << "out of boundary" << endl;
                }
            }
            
        }

        // find the optimal solution
        // find a state with minimal g val in start_set

        Node* curr = nullptr;
        cout << "candidate_set.size(): " << candidate_set.size() << endl;

        // for (auto it = candidate_set.begin(); it != candidate_set.end(); ++it) {
        //     // if (curr_time == 1) {
        //         // cout << "curr_time: " << curr_time << ", it->second->t: " << it->second->t << ", it->g" << it->first << endl;
        //     // }
        //     if (grid[it->second]->t - curr_time > grid[it->second]->g) {
        //         curr = grid[it->second];
        //         break;
        //     }
        // }
        curr = grid[candidate_set.begin()->second];

        // backtrace
        if (curr == nullptr) {
            cout << "no path found" << endl;
            return;
        }

        while (curr != nullptr) {
            path.push_back(curr);
            curr = curr->parent;
            cout << curr->x << " " << curr->y << endl;
        }

        pathSize = path.size();
        cout << "path size: " << pathSize << endl;

        foundPath = true;

        action_ptr[0] = path[pathIndex]->x;
        action_ptr[1] = path[pathIndex]->y;

    } else {

        if (pathIndex != pathSize) {
            action_ptr[0] = path[pathIndex]->x;
            action_ptr[1] = path[pathIndex]->y;
            pathIndex++;
        }

    }

    return;
}