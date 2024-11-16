#ifndef AGENT_H
#define AGENT_H

#include "SharedEnv.h"
#include "RRA_star/RRAstar.h"

class Agent {
public:
    const int id;
    int p {-1};
    int nextLoc {-1};

    Agent(int _id, SharedEnvironment* _env): id(_id), heuristic(_env), env(_env) { }

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
    RRAstar heuristic;
    int goal {-1};
    int goalDist {-1};

    bool validateMove(int loc1, int loc2) const;
};

#endif // AGENT_H