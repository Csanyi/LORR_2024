#ifndef AGENT_H
#define AGENT_H

#include "SharedEnv.h"
#include "RRA_star/RRAstar.h"

class Agent {
public:
    const int id;
    double p;
    int nextLoc {-1};

    Agent(int _id, double _p, SharedEnvironment* _env): id(_id), p(_p), heuristic(_env), env(_env) { }

    void setGoal();
    int getDist(int loc, int dir);
    int getLoc() const;
    int getDir() const;
    bool isNewGoal() const;

    std::vector<std::pair<int,int>> getNeighborsWithDist();
    std::vector<std::pair<int,int>> getNeighborsWithUnknownDist() const;

private:
    SharedEnvironment* env;
    RRAstar heuristic;
    int goal {-1};

    bool validateMove(int loc1, int loc2) const;
};

#endif // AGENT_H