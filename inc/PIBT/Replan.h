#ifndef REPLAN_H
#define REPLAN_H

#include <vector>
#include <list>
#include "SharedEnv.h"
#include "PIBT/Agent2.h"

class Replan {
public:
    Replan(SharedEnvironment* _env, std::vector<std::vector<int>>& _reservations): env(_env), reservations(_reservations), reservationsCopy(_reservations) {}

    bool replan(std::vector<Agent2*>& agents);

private:
    SharedEnvironment* env;
    std::vector<std::vector<int>>& reservations;
    std::vector<std::vector<int>> reservationsCopy;
    pair<bool,list<pair<int,int>>> single_agent_plan(int id, int start,int start_direct, int end, int end_dir);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction, int time);
    bool validateMove(int loc,int loc2);
};

#endif // REPLAN_H
