#include <random>
#include <set>
#include <algorithm>
#include <chrono>
#include "PIBT/PIBT.h"

void PIBT::initialize() {
    agentsPerThread = env->num_of_agents / threadCnt;
    remainingAgents = env->num_of_agents % threadCnt;

    prevReservations.resize(env->map.size(), -1);
    nextReservations.resize(env->map.size(), -1);

    maputils.eraseDeadEnds();

    int areaDist = (env->map.size() < 2048) ? env->cols + 1 : 5;
    maputils.divideIntoAreas(areaDist, false);
    maputils.calculateDistanceBetweenAreas();

    agents.reserve(env->num_of_agents);  
    agentsById.reserve(env->num_of_agents);    
  
    for (int i {0}; i < env->num_of_agents; ++i) {
        Agent* agent = new Agent(i, env, &maputils);
        agents.push_back(agent);
        agentsById.push_back(agent);
    }
}

void PIBT::nextStep(int timeLimit, std::vector<Action>& actions) {
    const TimePoint startTime  {std::chrono::steady_clock::now()};
    const TimePoint endTime {startTime + std::chrono::milliseconds(timeLimit - 20)};

    for (auto& agent : agents) {
        prevReservations[agent->getLoc()] = agent->id;
        if (maputils.deadEndMap[agent->getLoc()] == MapUtils::DEADLOC) { agent->boostPriority(); }
    }

    calculateGoalDistances();

    std::sort(agents.begin(), agents.end(), [](const Agent* a, const Agent* b) {
        if (a->p == b->p) {
            return a->id < b->id;
        }
        return a->p < b->p;
    });
    
    for (auto& agent : agents) {
        if (std::chrono::steady_clock::now() > endTime) {
            std::cout << "-------- Out of time during nextLoc (t: " << env->curr_timestep << ") at agent: " << agent->id << '\n';
            break;
        }
        if (agent->nextLoc == -1) {
            getNextLoc(agent, nullptr);
        }
    }

    actions = std::vector<Action>(env->num_of_agents, Action::NA);
    std::vector<bool> visited(env->num_of_agents, false);
    for (auto& a : agents) {
        if (actions[a->id] == Action::NA) {
            actions[a->id] = getNextAction(actions, visited, a);
        }
        a->nextLoc = -1;
    }

    std::fill(prevReservations.begin(), prevReservations.end(), -1);
    std::fill(nextReservations.begin(), nextReservations.end(), -1);
}

bool PIBT::getNextLoc(Agent* const a, const Agent* const b) {
    auto neighbors = a->getNeighborsWithDist();
    std::sort(neighbors.begin(), neighbors.end(), [this, &a, &b](const std::pair<int, int>& n1, const std::pair<int, int>& n2) {
        if (maputils.deadEndMap[n1.first] == MapUtils::DEADLOC && a->getGoal() != n1.first) { return false; }

        if (n1.second == n2.second) {
            return prevReservations[n1.first] == -1;
        }
        return n1.second < n2.second;
    });

    for (const auto& neighbor : neighbors) {
        if (nextReservations[neighbor.first] != -1) { continue; }

        if (b != nullptr && prevReservations[neighbor.first] == b->id) { continue; }

        a->nextLoc = neighbor.first;
        nextReservations[neighbor.first] = a->id;

        int otherAgent {prevReservations[neighbor.first]};

        if (otherAgent != -1 && agentsById[otherAgent]->nextLoc == -1) {
            if (getNextLoc(agentsById[otherAgent], a) == false) { continue; }
        }

        return true;
    }

    a->nextLoc = a->getLoc();
    nextReservations[a->nextLoc] = a->id;
    return false;
}

Action PIBT::getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent* const a) {
    if (a->nextLoc == -1) { return Action::W; }
    Action action;
    int dir;
    int diff {a->nextLoc - a->getLoc()};
    
    if (diff == 0) {
        return Action::W;
    }
    else if (diff == 1) {
        dir = 0;
    }
    else if (diff == env->cols) {
        dir = 1;
    }
    else if (diff == -1) {
        dir = 2;
    }
    else if (diff == -env->cols) {
        dir = 3;
    }
    else {
        assert(false);
    }

    if (a->getDir() == dir) {
        int otherAgent {prevReservations[a->nextLoc]};
        assert(otherAgent != a->id);

        if (otherAgent != -1 && actions[otherAgent] == Action::NA && visited[otherAgent] == false) {
            visited[a->id] = true;
            actions[otherAgent] = getNextAction(actions, visited, agentsById[otherAgent]);
            action = nextReservations[a->nextLoc] == a->id ? Action::FW : Action::W;
        }
        else if (otherAgent != -1 && actions[otherAgent] == Action::NA && visited[otherAgent] == true) {
            action = Action::FW;
        }
        else if (otherAgent != -1 && actions[otherAgent] != Action::FW) {
            action = Action::W;
        }
        else {
            action = Action::FW;
        }
    }
    else {
        int incr {dir - a->getDir()};
        if (incr == 1 || incr == -3 || incr == 2 || incr == -2) {
            action = Action::CR;
        }
        else if (incr == -1 || incr == 3) {
            action = Action::CCR;
        }
    }

    if (action != Action::FW) {
        if (nextReservations[a->nextLoc] == a->id) {
            nextReservations[a->nextLoc] = -1;
        }
        nextReservations[a->getLoc()] = a->id;
    }

    return action;
}

void PIBT::calculateGoalDistances() {
    std::vector<std::thread> threads;
    int remaining {remainingAgents};
    int from {0};
    int to;

    for (int i {0}; i < threadCnt; ++i) {
        to = from + agentsPerThread + (remaining-- > 0 ? 1 : 0);

        threads.emplace_back([this, from, to]() {
            for (int j {from}; j < to; ++j) {
                if (agents[j]->isNewGoal()) {
                    agents[j]->setGoal(); 
                }

                agents[j]->calculateNeighborDists();
            }
        });

        from = to;
    }

    for (auto t = threads.begin(); t != threads.end(); ++t) {
        t->join();
    }
}

double PIBT::countClosedNodeAvg() const {
    int closedSum {0};
    int taskSum {0};
    for (auto& agent : agents) {
        closedSum += agent->getClosedCnt();
        taskSum += agent->getCompletedTaskCnt();
    }

    return static_cast<double>(closedSum) / taskSum;
}

PIBT::~PIBT() {
    for (auto a : agents) {
        delete a;
    }
}