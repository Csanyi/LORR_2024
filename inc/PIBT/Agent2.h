#ifndef AGENT2_H
#define AGENT2_H

#include "SharedEnv.h"
#include "RRA_star/RRAstar.h"
#include "reduce_map/ReduceMap.h"

class Agent2 {
public:
    const int id;
    int p {-1};
    int nextLoc {-1};

    Agent2(int _id, SharedEnvironment* _env, ReduceMap& _reduce): id(_id), heuristic(_env), env(_env), reduce(_reduce) { }

    void setGoal();
    int getDist(int loc, int dir);
    int getLoc() const;
    int getDir() const;
    bool isNewGoal() const;
    void boostPriority() { p -= 10000; }
    void resetPriority() { p = goalDist; }

    std::vector<std::pair<int,int>> getNeighborsWithDist();
    std::vector<std::pair<int,int>> getNeighborsWithUnknownDist() const;

private:
    SharedEnvironment* env;
    ReduceMap reduce;
    RRAstar heuristic;
    int goal {-1};
    int goalDist {-1};
    int nextArea;

    bool validateMove(int loc1, int loc2) const;
    void initializeHeuristic(int areaFrom, int areaTo);
};

#endif // AGENT2_H