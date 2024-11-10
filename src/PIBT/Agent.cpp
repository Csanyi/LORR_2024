#include <chrono>   
#include "PIBT/Agent.h"

void Agent::setGoal() {
    if (env->goal_locations[id].empty()) {
        goal = env->curr_states[id].location;
    }
    else {
        goal = env->goal_locations[id].front().first;
    }

    // auto start {std::chrono::steady_clock::now()};

    heuristic.reset();
    heuristic.initialize(env->curr_states[id].location, env->curr_states[id].orientation, goal);

    // auto end {std::chrono::steady_clock::now()};
    // auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    // if (duration.count() > 10) 
    // {
    //     std::cout << "******** Heuristic for agent "<< id << " took " << duration.count() << " ms (" << env->curr_states[id].location
    //         << ", " << goal << ")\n";
    // }
}

int Agent::getDist(int loc, int dir) {
    // auto start {std::chrono::steady_clock::now()};

    return heuristic.abstractDist(loc, dir);

    // auto end {std::chrono::steady_clock::now()};
    // auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    // if (duration.count() > 10) {
    //     std::cout << "******** Heuristic for agent "<< id << " took " << duration.count() << " ms (" << env->curr_states[id].location
    //         << ", " << goal << ")\n";
    // }

    // return result;
}

int Agent::getLoc() const {
    return env->curr_states[id].location;
}

int Agent::getDir() const {
    return env->curr_states[id].orientation;
}

bool Agent::isNewGoal() const {
    if (env->goal_locations[id].empty()) {
        return goal == -1;
    }

    return goal != env->goal_locations[id].front().first;
}

std::vector<std::pair<int,int>> Agent::getNeighborsWithDist() {
    std::vector<std::pair<int,int>> neighbours;
    int loc {getLoc()};
    int dir {getDir()};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], getDist(candidates[i], i)));
        }
    }

    neighbours.emplace_back(std::make_pair(loc, getDist(loc, dir)));

    return neighbours;
}

std::vector<std::pair<int,int>> Agent::getNeighborsWithUnknownDist() const {
    std::vector<std::pair<int,int>> neighbours;
    int loc {getLoc()};
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i{0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc) && !heuristic.isDistKnown(candidates[i], i)) {
            neighbours.emplace_back(std::make_pair(candidates[i], i));
        }
    }

    return neighbours;
}

bool Agent::validateMove(int loc1, int loc2) const {
    int loc_x {loc1 / env->cols};
    int loc_y {loc1 % env->cols};
    if (env->map[loc1] == 1 || loc_x >= env->rows || loc_y >= env->cols || loc_x < 0 || loc_y < 0) {
        return false;
    }

    int loc2_x {loc2 / env->cols};
    int loc2_y {loc2 % env->cols};
    if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1) {
        return false;
    }

    return true;
}