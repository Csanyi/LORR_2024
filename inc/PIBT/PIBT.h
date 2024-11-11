#ifndef PIBT_H
#define PIBT_H

#include <vector>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "PIBT/Agent.h"
#include "reduce_map/ReduceMap.h"

class PIBT {
public:
    PIBT(SharedEnvironment* _env): env(_env), reduce(_env) { }
    ~PIBT();

    void initialize();
    void nextStep(int timeLimit, std::vector<Action>& actions);

private:
    SharedEnvironment* env;
    std::vector<Agent*> agentsById;
    std::vector<Agent*> agents;
    std::vector<int> prevReservations;
    std::vector<int> nextReservations;
    ReduceMap reduce;

    bool getNextLoc(Agent* const a, const Agent* const b);
    Action getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent* const a);

    void setGoalsParallel();
    void setGoals(const TimePoint& endTime);
};

#endif // PIBT_H