#include <chrono>
#include "MAPFPlanner.h"
#include"const.h"

void MAPFPlanner::initialize(int preprocess_time_limit) {
    auto start {std::chrono::steady_clock::now()};

    pibt.initialize();

    auto end {std::chrono::steady_clock::now()};
    auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    std::cout << "******** Initialization took " << duration.count() << " ms ********\n";
}

void MAPFPlanner::plan(int time_limit, vector<Action>& actions) {
    auto start {std::chrono::steady_clock::now()};

    auto limit {time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now()
         - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE};

    pibt.nextStep(limit, actions);

    auto end {std::chrono::steady_clock::now()};
    auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(end - start)};

    std::cout << "******** Planning at timestep " << env->curr_timestep << " took " << duration.count() << " ms ********\n";
}
