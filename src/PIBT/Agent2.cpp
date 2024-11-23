#include <chrono>   
#include "PIBT/Agent2.h"

void Agent2::setGoal() {
    if (env->goal_locations[id].empty()) {
        goal = getLoc();
    }
    else {
        goal = env->goal_locations[id].front().first;
    }

    int areaFrom {reduce.areaMap[getLoc()]};
    int areaTo {reduce.areaMap[goal]};

    initializeHeuristic(areaFrom, areaTo);

    goalDist = reduce.basePointDistances[areaFrom][areaTo].first + heuristic.abstractDist(getLoc(), getDir());
    p = goalDist;
}

int Agent2::getDist(int loc, int dir) {
    int areaFrom {reduce.areaMap[getLoc()]};

    if (areaFrom == nextArea && nextArea != -1) {
        int areaTo {reduce.areaMap[goal]}; 
        initializeHeuristic(areaFrom, areaTo);
    } 

    return heuristic.abstractDist(loc, dir);
}

int Agent2::getLoc() const {
    return env->curr_states[id].location;
}

int Agent2::getDir() const {
    return env->curr_states[id].orientation;
}

bool Agent2::isNewGoal() const {
    if (env->goal_locations[id].empty()) {
        return goal == -1;
    }

    return goal != env->goal_locations[id].front().first;
}

std::vector<std::pair<int,int>> Agent2::getNeighborsWithDist() {
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

std::vector<std::pair<int,int>> Agent2::getNeighborsWithUnknownDist() const {
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

bool Agent2::validateMove(int loc1, int loc2) const {
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

void Agent2::initializeHeuristic(int areaFrom, int areaTo) {
    heuristic.reset();

    if (areaFrom == areaTo) {
        nextArea = -1;
        heuristic.initialize(getLoc(), getDir(), goal);
    } 
    else {
        nextArea = reduce.basePointDistances[areaFrom][areaTo].second;
        heuristic.initialize(getLoc(), getDir(), nextArea, reduce);
        //heuristic.initialize(getLoc(), getDir(), reduce.basePoints[nextArea]);
    }
}