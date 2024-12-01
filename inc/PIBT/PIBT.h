#ifndef PIBT_H
#define PIBT_H

#include <vector>
#include <boost/thread.hpp>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "PIBT/Agent2.h"
#include "reduce_map/ReduceMap.h"

class PIBT {
public:
    PIBT(SharedEnvironment* _env): env(_env), reduce(_env) { }
    ~PIBT();

    void initialize();
    void nextStep(int timeLimit, std::vector<Action>& actions);

private:
    const int threadCnt {static_cast<int>(boost::thread::hardware_concurrency())};
    int agentsPerThread;
    int remainingAgents;
    SharedEnvironment* env;
    std::vector<Agent2*> agentsById;
    std::vector<Agent2*> agents;
    std::vector<int> prevReservations;
    std::vector<int> nextReservations;
    ReduceMap reduce;

    bool getNextLoc(Agent2* const a, const Agent2* const b);
    Action getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent2* const a);

    void calculateGoalDistances();
    void setGoals(const TimePoint& endTime);
};

#endif // PIBT_H