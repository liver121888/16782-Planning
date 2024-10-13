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

struct CompareNode {
    bool operator()(Node* n1, Node* n2) {
        // more false, more priority
        if (n1->f != n2->f)
            return n1->f > n2->f;
        else
            return n1->g < n2->g;
    }
};

// heuristics, hash
priority_queue<Node*, vector<Node*>, CompareNode> open_pq;

unordered_set<string> closed_set;

unordered_map<string, pair<Node*, int>> grid;

// end_set is storing all the target states with time
unordered_map<string, vector<int>> end_set;

// cost, target t, hash
vector<pair<int, string>> candidate_set;

Node* curr;
stack<Node*> path;

int pathSize = 0;
bool switchState = false;

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

    // if (plannerState < 2) {
    //     cout << "++++++++++ CHECK ++++++++++" << endl;
    //     cout << "plannerState: " << plannerState << ", curr_time: " << curr_time << endl;
    //     cout << "++++++++++ CHECK ++++++++++" << endl;
    // }

    if (!switchState) {

        // Record the start time
        auto start = std::chrono::high_resolution_clock::now();

        Node* dummy = new Node(-1, -1, 0, 0, 0, 0, "", "");
        grid[dummy->hash] = make_pair(dummy, dummy->version);
        Node* start_state = new Node(robotposeX, robotposeY, 0, 0, 0, 0, dummy->hash, "");
        start_state->leastCostParentHash = start_state->hash;
        start_state->updateMulti(target_steps, target_traj);

        // open_pq
        open_pq.push(start_state);
        grid[start_state->hash] = make_pair(start_state, start_state->version);

        // ignore some starting states to save planning time
        for (int i = 6*target_steps/13; i < target_steps; i++) {
            // dummy goal state
            // Node* goal_state = new Node(target_traj[i], target_traj[i+target_steps], 0, INFINITE_COST, 0, "", "");
            // Node::printNode(*goal_state);
            end_set[to_string(target_traj[i]) + " " + to_string(target_traj[i+target_steps])].push_back(i);
            // delete goal_state;
            // cout << i << ", start_hash: " << goal_state->hash() << endl;

        }

        // cout << "end_set.size(): " << end_set.size() << ", open_pq.size(): " << open_pq.size() << endl;
        
        // cout << "======== THE LOOP ========" << endl;
        while (end_set.size() > 0 && open_pq.size() > 0) {
            
            Node* topPriorityNode = open_pq.top();
            open_pq.pop();

            // Skip outdated nodes by checking version
            if (topPriorityNode->version < grid[topPriorityNode->hash].second) {
                continue; // Ignore outdated node
            }
            
            // cout << "topPriorityNode: " << endl;
            // Node::printNode(*topPriorityNode);

            closed_set.insert(topPriorityNode->hash);       

            if (end_set.find(topPriorityNode->hash) != end_set.end()) {

                vector<int> times = end_set[topPriorityNode->hash];
                
                for (int time : times) {

                    cout << "target t: " << time << ", node t: " << topPriorityNode->t << ", node hash: " << topPriorityNode->hash << endl;

                    if (time >= topPriorityNode->t) {

                        cout << "goal reachable" << endl;
                        candidate_set.push_back(make_pair(time, topPriorityNode->hash));
                    } else {
                        // cout << "goal x" << endl;
                    }

                }

                end_set.erase(topPriorityNode->hash);
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
                        Node* new_state = new Node(newx, newy, topPriorityNode->t + 1, INFINITE_COST, 0, 0,
                                                        topPriorityNode->hash, topPriorityNode->leastCostParentHash);

                        int least_cost = map[GETMAPINDEX(grid[topPriorityNode->leastCostParentHash].first->x,
                                                    grid[topPriorityNode->leastCostParentHash].first->y,x_size,y_size)];
                        // <: wait at start, <=: wait at goal
                        string least_cost_hash = new_cost <= least_cost ? new_state->hash : topPriorityNode->leastCostParentHash;
                        new_state -> leastCostParentHash = least_cost_hash;
                        new_state->updateMulti(target_steps, target_traj);

                        // if not closed
                        if (closed_set.find(new_state->hash) == closed_set.end()) {

                            // cout << "not closed" << endl;    
                            if (new_state->g > topPriorityNode->g + new_cost) {
                                new_state->g = topPriorityNode->g + new_cost;
                                // cout << "gg: " << new_state->g << endl;
                                new_state->update();

                                bool foundNode = false;
                                if (grid.find(new_state->hash) != grid.end()) {
                                    foundNode = true;
                                    int old_version = grid[new_state->hash].second;
                                    Node* old_node = grid[new_state->hash].first;

                                    // Increment the version for the new node
                                    int new_version = old_version + 1;
                                    new_state->version = new_version;

                                    open_pq.push(new_state);
        
                                    // Update the map with new node pointer and version
                                    grid[new_state->hash] = make_pair(new_state, new_state->version);
                                }
       
                                if (!foundNode) {
                                    // cout << "not found in open either, push to open" << endl;

                                    // cout << "f: "<< new_state->f << endl;
                                    // insert into open
                                    open_pq.push(new_state);
                                    grid[new_state->hash] = make_pair(new_state, new_state->version);
                                }

                            } else {
                                // always better
                                // cout << "not better" << endl;
                                // delete new_state;
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
        // cout << "======== THE LOOP ========" << endl;

        // Record the end time
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Time elapsed: " << duration.count() << " ms" << std::endl;
        int time_now = duration.count()/1000;
        std::cout << "time_now: " << time_now << " s" << std::endl;

        // find the optimal solution
        // find a state with minimal g val in start_set

        // cout << "candidate_set.size(): " << candidate_set.size() << endl;
        Node* curr = nullptr;
        int minPathCost = INFINITE_COST;
        int minCostWaitingTime = 0;
        int minLeastCost = 0;

        for (auto aPair : candidate_set) {

            int targetTime = aPair.first;
            Node* tmp = grid[aPair.second].first;

            // cout << "candidate g: " << g << ", t: " << t << ", hash: " << hash << endl;
            if (targetTime - time_now >= tmp->t) {

                // feasible
                Node* leastCostParent = grid[tmp->leastCostParentHash].first;
                int leastCost = map[GETMAPINDEX(leastCostParent->x,leastCostParent->y,x_size,y_size)];
                int waitingTime = targetTime - time_now  - tmp->t;
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
            // plannerState = 2;
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            switchState = true;

        } else {

            Node* head = curr;
            cout << "path found" << endl;
            cout << "head->x: " << head->x << ", head->y: " << head->y << endl;
            cout << "head->t: " << head->t << ", minCostWaitingTime: " << minCostWaitingTime << ", minLeastCost: " << minLeastCost << endl;
            cout << "head position: " << head->hash << ", leastCost position: " << head->leastCostParentHash << endl;
            cout << "minPathCost: " << minPathCost << endl;
            
            while (curr->hash != "-1 -1") {

                path.push(curr);
                // cout << curr->x << " " << curr->y << endl;

                if (head->leastCostParentHash == curr->hash && minCostWaitingTime > 0) {
                    minCostWaitingTime--;
                } else {
                    curr = grid[curr->parentHash].first;
                }

            }

            // remove dummy node
            path.pop();

            // pathSize = path.size();
            // cout << "path size: " << pathSize << endl;

            switchState = true;

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
        } else {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
        }
    }

    return;
}