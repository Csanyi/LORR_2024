#include "PIBT/Replan.h"
#include <bits/stdc++.h>

struct AstarNode {
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};

struct cmp {
    bool operator()(AstarNode* a, AstarNode* b) {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};

bool Replan::replan(std::vector<Agent2*>& agents) {
    std::vector<Agent2*> agentsCopy;
    std::random_device rd;
    std::mt19937 g(rd());

    for (int i {0}; i < agents.size(); ++i) {
        Agent2* agent = new Agent2(agents[i]->id, env, nullptr);
        agent->locations = agents[i]->locations;
        agent->p = i;
        agentsCopy.push_back(agent);
    }

    std::shuffle(agentsCopy.begin(), agentsCopy.end(), g);

    for (int i {1}; i < reservationsCopy.size(); ++i) {
        for (auto a : agentsCopy) {
            auto start = a->locations.front();
            auto goal = a->locations.back();
            if (start.first == goal.first) { continue; }
            else if (i < a->locations.size()) {
                reservationsCopy[i][a->locations[i].first] = -1;
            }
            else {
                reservationsCopy[i][a->locations.back().first] = -1;
            }
        }
    }

    for (auto a : agentsCopy) {
        auto start = a->locations.front();
        auto goal = a->locations.back();
        if (start.first == goal.first) { continue; }
        auto result = single_agent_plan(a->id, start.first, start.second, goal.first, goal.second);
        if (result.first == false) {return false;}
        a->locations.resize(1);
        std::copy(result.second.begin(), result.second.end(), std::back_inserter(a->locations));
    }

    int prevSum {0};
    int currSum {0};
    for (int i {0}; i < agents.size(); ++i) {
        agentsCopy[i]->goalTime = agentsCopy[i]->locations.size() - 1;
        currSum += agentsCopy[i]->goalTime;
        prevSum += agents[i]->goalTime;
    }

    if (currSum < prevSum) {
        reservations = reservationsCopy;

        for (auto a : agentsCopy) {
            if (agents[a->p]->locations.front() != agents[a->p]->locations.back()) {
                agents[a->p]->locations = a->locations;
                agents[a->p]->goalTime = a->goalTime;
            }
        }
    }

    return true;
}

pair<bool,list<pair<int,int>>> Replan::single_agent_plan(int id, int start,int start_direct,int end, int end_dir) {
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;
    bool success {false};

    while (!open_list.empty()) {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end && curr->direction == end_dir) {
            for (int i{curr->g+1}; i < reservationsCopy.size(); ++i) {
                reservationsCopy[i][curr->location] = id;
            }
            while(curr->parent!=nullptr) {
                path.emplace_front(make_pair(curr->location, curr->direction));
                assert(reservationsCopy[curr->g][curr->location] == -1);
                reservationsCopy[curr->g][curr->location] = id;
                curr = curr->parent;
            }
            success = true;
            break;
        }

        if (reservationsCopy.size() <= curr->g + 1) {
            reservationsCopy.push_back(std::vector<int>(env->map.size(), -1));
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction, curr->g + 1);
        for (const pair<int,int>& neighbor: neighbors) {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end()) {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g) {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes) {
        delete n.second;
    }
    all_nodes.clear();
    return {success, path};
}


int Replan::getManhattanDistance(int loc1, int loc2) {
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool Replan::validateMove(int loc, int loc2) {
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}

list<pair<int,int>> Replan::getNeighbors(int location, int direction, int time) {
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1, location + env->cols, location - 1, location - env->cols };
    int forward = candidates[direction];
    int new_direction = direction;

    if (forward>=0 && forward < env->map.size() && validateMove(forward,location)) {
        int id = reservationsCopy[time-1][forward];
        int id2 = reservationsCopy[time][location];
        if (reservationsCopy[time][forward] == -1 && (id == -1 || id != id2)) {
            neighbors.emplace_back(make_pair(forward,new_direction));
        }
    }
    
    if (reservationsCopy[time][location] == -1) {
        //turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        neighbors.emplace_back(make_pair(location,new_direction));
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        neighbors.emplace_back(make_pair(location,new_direction));
        neighbors.emplace_back(make_pair(location,direction)); //wait
    }

    return neighbors;
}
