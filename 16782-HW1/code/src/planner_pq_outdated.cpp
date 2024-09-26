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

// // Custom hash function for pair<int, string>
// struct PairHash {
//     template <typename T1, typename T2>
//     std::size_t operator()(const std::pair<T1, T2>& p) const {
//         std::size_t h1 = std::hash<T1>()(p.first);        // Hash for int
//         std::size_t h2 = std::hash<T2>()(p.second);       // Hash for string
//         return h1 ^ (h2 << 1); // Combine hashes using XOR and bit shifting
//     }
// };

// // Custom equality comparator for pair<int, string>
// struct PairEqual {
//     bool operator()(const std::pair<int, std::string>& p1, const std::pair<int, std::string>& p2) const {
//         return p1.first == p2.first && p1.second == p2.second;
//     }
// };




// 8-connected grid
int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

struct CompareNode {
    bool operator()(Node* n1, Node* n2) {
        return n1->f > n2->f;
    }
};

// heuristics, hash
priority_queue<Node*, vector<Node*>, CompareNode> open_pq;

unordered_set<string> closed_set;

unordered_map<string, pair<Node*, int>> grid;

// end_set is storing all the target states with time
unordered_map<string, vector<int>> end_set;

// cost, target t, hash
// unordered_set<pair<int, string>, PairHash, PairEqual> candidate_set;

Node* curr;
stack<Node*> path;

int pathSize = 0;
int plannerState = 0;

int checkTimeCount = 0;
int checkTime = 5;

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

    if (checkTimeCount < checkTime) {
        cout << "plannerState: " << plannerState << ", curr_time: " << curr_time << endl;
    }

    if (plannerState == 0) {

        Node* dummy = new Node(-1, -1, 0, 0, 0, 0, "", "");
        grid[dummy->hash()] = make_pair(dummy, dummy->version);
        Node* start_state = new Node(robotposeX, robotposeY, 0, 0, 0, 0, dummy->hash(), "");
        start_state->leastCostParentHash = start_state->hash();
        start_state->updateMulti(target_steps, target_traj);

        // open_pq
        open_pq.push(start_state);
        grid[start_state->hash()] = make_pair(start_state, start_state->version);

        for (int i = 0; i < target_steps; i++) {
            // dummy goal state
            // Node* goal_state = new Node(target_traj[i], target_traj[i+target_steps], 0, INFINITE_COST, 0, "", "");
            // Node::printNode(*goal_state);
            end_set[to_string(target_traj[i]) + " " + to_string(target_traj[i+target_steps])].push_back(i);
            // delete goal_state;
            // cout << i << ", start_hash: " << goal_state->hash() << endl;

        }

        cout << "===================" << endl;
        cout << "end_set.size(): " << end_set.size() << ", open_pq.size(): " << open_pq.size() << endl;

        while (end_set.size() > 0 && open_pq.size() > 0) {

            // for (auto it = grid.begin(); it != grid.end(); ++it) {
            //     cout << it->first << " " << it->second << endl;
            // }
            
            Node* topPriorityNode = open_pq.top();
            open_pq.pop();
            if (outdated_set.find(topPriorityNode) != outdated_set.end()) {
                // found a node that is outdated, skip
                // cout << "outdated: " << topPriorityNode << outdated_set.size() << endl;
                outdated_set.erase(topPriorityNode);
                delete topPriorityNode;
                continue;
            }
            cout << "topPriorityNode: " << endl;
            Node::printNode(*topPriorityNode);

            closed_set.insert(topPriorityNode->hash());

            // cout << "check_hash: " << topPriorityNode->hash() << endl;
            // cout << "check_h: " << topPriorityNode->h << endl;

            // for (auto it = open_pq.begin(); it != open_pq.end(); ++it)
            // {
            //     cout << it->first << " " << it->second << endl;
            // }            

            if (end_set.find(topPriorityNode->hash()) != end_set.end()) {

                vector<int> times = end_set[topPriorityNode->hash()];
                
                for (int time : times) {

                    cout << "target t: " << time << ", node t: " << topPriorityNode->t << ", node hash: " << topPriorityNode->hash() << endl;

                    if (time > topPriorityNode->t) {

                        cout << "goal reachable" << endl;
                        candidate_set.insert(make_pair(time, topPriorityNode->hash()));
                    } else {
                        cout << "goal x" << endl;
                    }

                }

                end_set.erase(topPriorityNode->hash());
                // cout << "end_set.size(): " << end_set.size() << endl;
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
                        Node* new_state = new Node(newx, newy, topPriorityNode->t + 1, topPriorityNode->g + new_cost, 0, 
                                                    topPriorityNode->hash(), topPriorityNode->leastCostParentHash);

                        // Node* new_state = new Node(newx, newy, topPriorityNode->t + 1, INFINITE_COST, 0, 
                        //                             topPriorityNode->hash(), topPriorityNode->leastCostParentHash);

                        int least_cost = map[GETMAPINDEX(grid[topPriorityNode->leastCostParentHash]->x,
                                                    grid[topPriorityNode->leastCostParentHash]->y,x_size,y_size)];
                        // <: wait at start, <=: wait at goal
                        string least_cost_hash = new_cost <= least_cost ? new_state->hash() : topPriorityNode->leastCostParentHash;
                        new_state -> leastCostParentHash = least_cost_hash;
                                                    
                        new_state->updateMulti(target_steps, target_traj);

                        // if not closed
                        if (closed_set.find(new_state->hash()) == closed_set.end()) {

                            // // cout << "not closed" << endl;    
                            // if (new_state->g > topPriorityNode->g + new_cost) {
                            //     new_state->g = topPriorityNode->g + new_cost;
                            //     // cout << "gg: " << new_state->g << endl;
                            //     new_state->updateMulti(target_steps, target_traj);

                            //     bool foundNode = false;
                            //     try {
                            //         grid.at(new_state->hash());
                            //         foundNode = true;
                            //         outdated_set.insert(grid[new_state->hash()]);
                            //         open_pq.push(new_state);
                            //         grid[new_state->hash()] = new_state;
                            //     } catch(const std::exception& e) {
                            //         // std::cerr << e.what() << '\n';
                            //     }      
                            //     if (!foundNode) {
                            //         // cout << "not found in open either, push to open" << endl;

                            //         // cout << "f: "<< new_state->f << endl;

                            //         // insert into open
                            //         open_pq.push(new_state);
                            //         grid[new_state->hash()] = new_state;
                            //     }

                            // } else {
                            //     // cout << "not better" << endl;
                            //     delete new_state;
                            // }



                            // cout << "not closed" << endl;    
                            bool foundNode = false;
                            try {
                                grid.at(new_state->hash());
                                foundNode = true;
                            } catch(const std::exception& e) {
                                // std::cerr << e.what() << '\n';
                            }              

                            if (foundNode) {
                                
                                Node* old_state = grid[new_state->hash()];
                                if (old_state->g > new_state->g) {
                                    
                                    // mark the node pointer as outdated
                                    outdated_set.insert(grid[old_state->hash()]);

                                    open_pq.push(new_state);
                                    grid[new_state->hash()] = new_state;

                                } else {
                                    // cout << "not better" << endl;
                                    delete new_state;
                                }

                            } else {
                                open_pq.push(new_state);
                                grid[new_state->hash()] = new_state;
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

        plannerState = 1;

        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;

    } else if (plannerState == 1) {

        // find the optimal solution
        // find a state with minimal g val in start_set

        // cout << "candidate_set.size(): " << candidate_set.size() << endl;
        Node* curr = nullptr;
        int minPathCost = INFINITE_COST;
        int minCostWaitingTime = 0;
        int minLeastCost = 0;

        for (auto it = candidate_set.begin(); it != candidate_set.end(); ++it) {

            int targetTime = it->first;
            Node* tmp = grid[it->second];

            // cout << "candidate g: " << g << ", t: " << t << ", hash: " << hash << endl;
            if (targetTime - curr_time >= tmp->t) {

                // feasible
                Node* leastCostParent = grid[tmp->leastCostParentHash];
                int leastCost = map[GETMAPINDEX(leastCostParent->x,leastCostParent->y,x_size,y_size)];
                int waitingTime = targetTime - tmp->t;
                int pathCost = waitingTime * leastCost + tmp->g;
                if (pathCost < minPathCost) {
                    minCostWaitingTime = waitingTime;
                    minPathCost = pathCost;
                    minLeastCost = leastCost;   
                    curr = tmp;
                }
            }

        }

        // backtrace
        if (curr == nullptr) {

            // cout << "no path found" << endl;

        } else {

            Node* head = curr;
            cout << "path found" << endl;
            cout << "head->t: " << head->t << ", minCostWaitingTime: " << minCostWaitingTime << endl;
            cout << "head position: " << head->hash() << ", leastCost position: " << head->leastCostParentHash << ", leastCost: " << minLeastCost << endl;
            cout << "minPathCost: " << minPathCost << endl;
            
            while (curr->hash() != "-1 -1") {

                path.push(curr);
                // cout << curr->x << " " << curr->y << endl;

                if (head->leastCostParentHash == curr->hash() && minCostWaitingTime > 0) {
                    minCostWaitingTime--;
                } else {
                    curr = grid[curr->parentHash];
                }

            }

            // remove dummy node
            path.pop();
            // remove start node
            // path.pop();
            // pathSize = path.size();
            // cout << "path size: " << pathSize << endl;

            plannerState = 2;

            curr = path.top();
            path.pop();
            // cout << curr->x << "fdsf" << curr->y << endl;
            action_ptr[0] = curr->x;
            action_ptr[1] = curr->y;
        }

    } else {

        if (!path.empty()) {
            Node* curr = path.top();
            path.pop();
            action_ptr[0] = curr->x;
            action_ptr[1] = curr->y;
        }
    }

    checkTimeCount++;
    return;
}