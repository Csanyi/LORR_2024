#include <random>
#include <set>
#include <algorithm>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "PIBT/PIBT.h"

void PIBT::initialize() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    std::set<double> uniqueValues;

    while (uniqueValues.size() < env->num_of_agents) {
        double randomValue = dis(gen);
        uniqueValues.insert(randomValue);
    }

    prevReservations.resize(env->map.size(), -1);
    nextReservations.resize(env->map.size(), -1);

    agents.reserve(env->num_of_agents);    
    int i {0};
    for (const auto& p : uniqueValues) {
        Agent* agent = new Agent(i, 1 - p, env);
        agents.push_back(agent);
        ++i;
    }

    reduce.eraseDeadEnds();
}

void PIBT::nextStep(int timeLimit, std::vector<Action>& actions) {
    TimePoint startTime  {std::chrono::steady_clock::now()};
    TimePoint endTime {startTime + std::chrono::milliseconds(timeLimit - 20)};

    for (auto& agent : agents) {
        prevReservations[agent->getLoc()] = agent->id;
    }

    if (env->map.size() > 9999) {
        setGoalsParallel(); 
    } 
    else {
        setGoals(endTime);
    }

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
    for (int i{0}; i < env->num_of_agents; ++i) {
        if (actions[i] == Action::NA) {
            actions[i] = getNextAction(actions, visited, agents[i]);
        }
        agents[i]->nextLoc = -1;
    }

    std::fill(prevReservations.begin(), prevReservations.end(), -1);
    std::fill(nextReservations.begin(), nextReservations.end(), -1);
}

bool PIBT::getNextLoc(Agent* const a, const Agent* const b) {
    auto neighbors = a->getNeighborsWithDist();
    std::sort(neighbors.begin(), neighbors.end(), [this](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        if (a.second == b.second) {
            return prevReservations[a.first] == -1;
        }
        return a.second < b.second;
    });

    for (const auto& neighbor : neighbors) {
        if (nextReservations[neighbor.first] != -1) { continue; }

        if (b != nullptr && prevReservations[neighbor.first] == b->id) { continue; }

        if (b != nullptr && reduce.deadEndMap[neighbor.first] == 2) { continue; }

        a->nextLoc = neighbor.first;
        nextReservations[neighbor.first] = a->id;

        int otherAgent {prevReservations[neighbor.first]};

        if (otherAgent != -1 && agents[otherAgent]->nextLoc == -1) {
            if (getNextLoc(agents[otherAgent], a) == false) {
                a->nextLoc = -1;
                nextReservations[neighbor.first] = -1;
                continue;
            }
        }

        return true;
    }

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
            actions[otherAgent] = getNextAction(actions, visited, agents[otherAgent]);
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

void PIBT::setGoalsParallel() {
    boost::asio::thread_pool pool(boost::thread::hardware_concurrency());

    for (auto& agent : agents) {
        if (agent->isNewGoal()) {
            boost::asio::post(pool, [agent]() { agent->setGoal(); });
        }
        else {
            auto neighbors = agent->getNeighborsWithUnknownDist();

            if (!neighbors.empty()) {
                boost::asio::post(pool, [agent, neighbors]() { 
                    for (auto& neighbor : neighbors) {
                        agent->getDist(neighbor.first, neighbor.second);
                    }
                });
            }
        }
    }

    pool.join();
}

void PIBT::setGoals(const TimePoint& endTime) {
    for (auto& agent : agents) {
        if (std::chrono::steady_clock::now() > endTime) {
            std::cout << "-------- Out of time during setGoal (t: " << env->curr_timestep << ") at agent: " << agent->id << '\n';
            break;
        }

        if (agent->isNewGoal()) {
            agent->setGoal();
        }
    }
}

PIBT::~PIBT() {
    for (auto a : agents) {
        delete a;
    }
}