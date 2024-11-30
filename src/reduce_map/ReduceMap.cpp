#include "reduce_map/ReduceMap.h"
#include <random>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

void ReduceMap::reduceMapStart()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows; x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols; y++)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : distance + 1; 
            reducedMap.push_back(distance);
        }
    }

    //From right to left, bottom to top
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc =  x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From top to bottom, left to right
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc =  x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From bottom to top, right to left
    for(int y = env->cols-1; y >= 0; y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
}

void ReduceMap::reduceMapUpdate()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows; x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols; y++)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From right to left, bottom to top
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc =  x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From top to bottom, left to right
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc =  x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From bottom to top, right to left
    for(int y = env->cols-1; y >= 0;y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
}

bool ReduceMap::reduceMap(bool keepSalient)
{
    bool changed = false;
    if(keepSalient)
    {
        for(int i = 0; i < reducedMap.size(); i++)
        {
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && areNeighboursTraversable(i) && ((countReducedVerticalNeighbours(i) + countReducedHorizontalNeighbours(i)) > 1))
            {
                changed = true;
                reducedMap[i] = 0;
            }
        }
    }
    else
    {
        vector<int> tmpReducedMap = reducedMap;
        for(int i = 0; i < reducedMap.size(); i++)
        {   
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && (countReducedVerticalNeighbours(i) + countReducedHorizontalNeighbours(i)) == 1)
            {
                changed = true;
                tmpReducedMap[i] = 0;
            }
        }
        reducedMap = tmpReducedMap;
    }

    reduceMapUpdate();
    return changed;
}

bool ReduceMap::areNeighboursTraversable(int location) const
{
    int x = location/env->cols;
    int y = location%env->cols;
    int horizontalNeighbourCount = countReducedHorizontalNeighbours(location);
    int verticalNeighbourCount = countReducedVerticalNeighbours(location);
    //Bridge Node
    if((verticalNeighbourCount == 0 && horizontalNeighbourCount == 2) || 
       (verticalNeighbourCount == 2 && horizontalNeighbourCount == 0))
    {
        return false;
    }
    //North + East + NorthEast
    if((location >= env->cols && reducedMap[location-env->cols] > 0) && 
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isNorthEastEmpty(location, x, y))
    {
        return false;
    }
    //South + East + SouthEast
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) && 
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isSouthEastEmpty(location, x, y))
    {
        return false;
    }
    //South + West + SouthWest
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) && 
       (y > 0 && reducedMap[location-1] > 0) &&
       isSouthWestEmpty(location, x, y))
    {
        return false;
    }
    //North + West + NorthWest
    if((location >= env->cols && reducedMap[location-env->cols] > 0) && 
       (y > 0 && reducedMap[location-1] > 0) &&
       isNorthWestEmpty(location, x, y))
    {
        return false;
    }
    return true;
}

bool ReduceMap::isNorthEastEmpty(int location, int x, int y) const
{
    return !(x > 0 && y + 1 < env->cols && reducedMap[location-env->cols+1] > 0);
}
bool ReduceMap::isSouthEastEmpty(int location, int x, int y) const
{
    return !(x + 1 < env->rows && y + 1 < env->cols && reducedMap[location+env->cols+1] > 0);
}
bool ReduceMap::isNorthWestEmpty(int location, int x, int y) const
{
    return !(x > 0 && y > 0 && reducedMap[location-env->cols-1] > 0);
}
bool ReduceMap::isSouthWestEmpty(int location, int x, int y) const
{
    return !(x + 1 < env->rows && y + 1 > 0 && reducedMap[location+env->cols-1] > 0);
}

int ReduceMap::countReducedVerticalNeighbours(int location) const
{
    int count = 0;
    if(location >= env->cols && reducedMap[location-env->cols] > 0) {count++;}
    if(location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) {count++;}
    return count;
}

int ReduceMap::countReducedHorizontalNeighbours(int location) const
{
    int count = 0;
    int y = location % env->cols;
    if(y > 0 && reducedMap[location-1] > 0) {count++;}
    if(y+1 < env->cols && reducedMap[location+1] > 0) {count++;}
    return count;
}

void ReduceMap::reduceMapWaypointsStart()
{
    for(int i = 0; i < env->map.size(); i++)
    {
        std::unordered_map<int,int> locationDistanceMap;
        reducedMapWaypoints.push_back(locationDistanceMap);
        if(reducedMap[i] > 0)
        {
            int locY = i % env->cols;
            if(i >= env->cols && reducedMap[i-env->cols] > 0)
            {
                int n = i-env->cols;
                reducedMapWaypoints[i][n] = 1;
            }
            if(locY + 1 < env->cols && reducedMap[i+1] > 0)
            {
                int e = i+1;
                reducedMapWaypoints[i][e] = 1;
            }
            if(i+env->cols < env->map.size() && reducedMap[i+env->cols] > 0)
            {
                int s = i+env->cols;
                reducedMapWaypoints[i][s] = 1;
            }
            if(locY > 0 && reducedMap[i-1] > 0)
            {
                int w = i-1;
                reducedMapWaypoints[i][w] = 1;
            }
        }
    }
}

bool ReduceMap::reduceReduceMapWaypoints(int distance)
{
    bool changed = false;
    for(int i = 0; i < env->map.size(); i++)
    {
        std::unordered_map<int,int> neighbours = reducedMapWaypoints[i];
        if(reducedMap[i] > 0 && neighbours.size() == 2)
        {
            vector<int> keys;
            for(auto kv : neighbours)
            {
                keys.push_back(kv.first);
            }
            int n0 = keys[0];
            int n1 = keys[1];
            int newCost = 999999;
            if (reducedMapWaypoints[n0].find(n1) == reducedMapWaypoints[n0].end())
            {
                int locToN1Cost = reducedMapWaypoints[i][n1];
                int n0ToLocCost = reducedMapWaypoints[n0][i];
                newCost = n0ToLocCost + locToN1Cost;
                if(newCost == 2 && (countReducedVerticalNeighbours(i) != 2) && (countReducedHorizontalNeighbours(i) != 2))
                {
                    newCost++;
                }
                if(newCost < distance)
                {
                    reducedMap[i] = 0;
                    reducedMapWaypoints[i].clear();
                    for (auto it = reducedMapWaypoints[n0].begin(); it != reducedMapWaypoints[n0].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n0].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n0][n1] = newCost;

                    for (auto it = reducedMapWaypoints[n1].begin(); it != reducedMapWaypoints[n1].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n1].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n1][n0] = newCost;
                    changed = true;
                }
            }
        }
    }
    return changed;
}

void ReduceMap::printReducedMap() const
{
    std::string blackBox = "■" ;
    std::string whiteBox = "□" ;

    for(int x = 0; x < env->rows; x++)
    {
        std::string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET); y++)
        {
            int loc = x * env->cols + y;
            if(areaMap[loc] == EMPTY)
            {
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            else 
            {
                temp += std::to_string(areaMap[loc]);
            }
        }
        std::cout << temp << '\n';
    }
    std::cout << "\n\n";
}

void ReduceMap::printReducedMapWaypoints() const
{
    std::string blackBox = "■" ;
    std::string whiteBox = "□" ;
    int cnt {0};

    for(int x = 0; x < env->rows; x++)
    {
        std::string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET); y++)
        {
            int loc = x * env->cols + y;
            std::unordered_map<int,int> tmpMap = reducedMapWaypoints[loc];
            if(reducedMapWaypoints[loc].size() == 0)
            { 
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            else
            {
                ++cnt;
                temp += whiteBox;
            }
        }
        std::cout << temp << '\n';
    }
    // std::cout << "Waypoint count: " << cnt << '\n';
    std::cout << "\n\n";
}

void ReduceMap::eraseDeadEnds() {
    deadEndMap = env->map;
    bool changed {true};
    int cnt {0};

    while (changed) {
        changed = false;
        for (int i {0}; i < deadEndMap.size(); ++i) {
            if (deadEndMap[i] > 0) { continue; }

            if (isDeadLoc(i)) {
                deadEndMap[i] = DEADLOC;
                changed = true;
                ++cnt;
                // std::cout << '(' << i / env->cols << ';' << i % env->cols << ")\n"; 
            }
        }
    }

    // std::cout << "erase cnt: " << cnt << '\n';
}

bool ReduceMap::isDeadLoc(int location) const {
    int cnt1 {0};
    int cnt2 {0};

    int x {location / env->cols};
    int y {location % env->cols};

    // right
    if (y + 1 < env->cols) {
        if (deadEndMap[location + 1] == 1) { ++cnt1; }
        else if (deadEndMap[location + 1] == DEADLOC) { ++cnt2; }
    }
    else { ++cnt1; }

    // left
    if (y - 1 >= 0) {
        if (deadEndMap[location - 1] == 1) { ++cnt1; }
        else if (deadEndMap[location - 1] == DEADLOC) { ++cnt2; }
    } 
    else { ++cnt1; }

    // top
    if (x - 1 >= 0) {
        if (deadEndMap[location - env->cols] == 1) { ++cnt1; }
        else if (deadEndMap[location - env->cols] == DEADLOC) { ++cnt2; }
    } 
    else { ++cnt1; }

    // bottom
    if (x + 1 < env->rows) {
        if (deadEndMap[location + env->cols] == 1) { ++cnt1; }
        else if (deadEndMap[location + env->cols] == DEADLOC) { ++cnt2; }
    } 
    else { ++cnt1; }

    return cnt1 == 3 || (cnt1 == 2 && cnt2 == 1);
}

void ReduceMap::hierarchyMapStart(int grid) {
    areaMap.resize(env->map.size(), 0);

    int cnt = 0;

    //Rows from left to right
    int gap_start = -1;
    for(int x = grid; x < env->rows; x += grid) {
        gap_start = -1;
        for(int y = 0; y < env->cols; ++y) {
            int loc = x * env->cols + y;
            // if we are in a gap and we reach the end of the row then put an exit 
            if (gap_start >= 0 && y == env->cols - 1) {
                areaMap[gap_start + (loc - gap_start) / 2] = 2;
                gap_start = -1;
                ++cnt;
            }
            // if we are in a gap and we reach a grid line then put an exit 
            // if (gap_start >= 0 && y % grid == 0) {
            //     gridMap[gap_start + (loc - gap_start) / 2] = 2;
            //     gap_start = -1;
            // }
            // if we are in a gap and we reach a wall then put an exit 
            else if (gap_start >= 0 && env->map[loc] == 1) {
                areaMap[gap_start + (loc - gap_start) / 2] = 2;
                gap_start = -1;
                ++cnt;
            }
            // if we are not in a gap and we reach a gap then start a gap 
            else if (gap_start < 0 && env->map[loc] == 0) {
                gap_start = loc;
            }
        }
    }
    //Cols from top to bottom
    for(int y = grid; y < env->cols; y += grid) {
        gap_start = -1;
        for(int x = 0; x < env->rows; ++x) {
            int loc = x * env->cols + y;
            // if we are in a gap and we reach the end of the column then put an exit 
            if (gap_start >=0 && x == env->rows - 1) {
                areaMap[(gap_start + (x - gap_start) / 2) * env->cols + y] = 2;
                gap_start = -1;
                ++cnt;
            }
            // if we are in a gap and we reach a grid line then put an exit 
            // if (gap_start >= 0 && x % grid == 0) {
            //     gridMap[(gap_start + (x - gap_start) / 2) * env->cols + y] = 2;
            //     gap_start = -1;
            // }
            // if we are in a gap and we reach a wall then put an exit 
            else if (gap_start >= 0 && env->map[loc] == 1) {
                areaMap[(gap_start + (x - gap_start) / 2) * env->cols + y] = 2;
                gap_start = -1;
                ++cnt;
            }
            // if we are not in a gap and we reach a gap then start a gap 
            else if (gap_start < 0 && env->map[loc] == 0) {
                gap_start = x;
            }
        }
    }

    std::cout << "Base point cnt: " << cnt << '\n';
}

void ReduceMap::divideIntoAreas(int distance) {
    markBasePoints(distance);
    createAreasAroundBasePoints();
}

void ReduceMap::markBasePoints(int distance) {
    areaMap.resize(env->map.size(), EMPTY);
    int id {0};

    for (int i {distance / 2}; i < env->rows; i += distance) {
        for (int j {distance / 2}; j < env->cols; j += distance) {
            if (markRandomPoint(i, j, distance / 3, id)) { ++id; }
        }
    }

    std::cout << "Base point cnt: " << id << '\n';
}

bool ReduceMap::markRandomPoint(int row, int col, int distance, int id) {
    std::vector<int> candidates, out;

    for (int i {max(row - distance, 0)}; i < min(row + distance, env->rows); ++i) {
        for (int j {max(col - distance, 0)}; j < min(col + distance, env->cols); ++j) {
            int loc {i * env->cols + j};
            if (env->map[loc] == 0) {
                candidates.push_back(loc);
            }
        }
    }

    if (!candidates.empty()) {
        std::sample(candidates.begin(), candidates.end(), std::back_inserter(out), 1, std::mt19937{std::random_device{}()});
        basePoints.push_back(out[0]);
        return true;
    }

    return false;
}

void ReduceMap::createAreasAroundBasePoints() {
    std::vector<std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNode::cmp>> open(basePoints.size());
    std::vector<std::unordered_map<int, DijkstraNode*>> closed(basePoints.size());
    std::vector<std::unordered_map<int, DijkstraNode*>> allNodes(basePoints.size());

    for (int i {0}; i < basePoints.size(); ++i) {
        DijkstraNode* s = new DijkstraNode(basePoints[i], EMPTY, 0);
        open[i].push(s);
        allNodes[i][s->location] = s;
    }

    int distance {0};
    bool running {true};
    while (running) {
        running = false;
        for (int i {0}; i < basePoints.size(); ++i) {
            if (open[i].empty()) { continue; }

            running = true;
            DijkstraNode* curr {open[i].top()};
            while (!open[i].empty() && curr->g == distance) {
                open[i].pop();
                closed[i][curr->location] = curr;

                for (const auto& neighbor: getNeighbors(curr->location, curr->parentLocation)) {
                    if (closed[i].find(neighbor.first) != closed[i].end()) { continue; }

                    if (allNodes[i].find(neighbor.first) != allNodes[i].end()) {
                        DijkstraNode* old = allNodes[i][neighbor.first];
                        if (curr->g + neighbor.second < old->g) {
                            old->g = curr->g + neighbor.second;
                            old->parentLocation = curr->location;
                        }
                    } 
                    else if (areaMap[neighbor.first] == EMPTY) {
                        DijkstraNode* nextNode = new DijkstraNode(neighbor.first, curr->location, curr->g + neighbor.second);
                        open[i].push(nextNode);
                        allNodes[i][nextNode->location] = nextNode;
                    }
                }

                if (areaMap[curr->location] == EMPTY) {
                    areaMap[curr->location] = i; // i == basePoint id
                    areaLocations[i].emplace(curr->location);
                }

                if (!open[i].empty()) {
                    curr = open[i].top();
                }
            }
        }

        ++distance;
    }
    
    for (auto& i : allNodes) {
        for (auto& n : i) {
            delete n.second;
        }
    }
            
    for (int i {0}; i < basePoints.size(); ++i) {
        createAreaBorders(i);
    }
}

void ReduceMap::createAreaBorders(int areaId) {
    for (const auto& areaLocation : areaLocations[areaId]) {
        for (const std::pair<int,int>& neighbor : getGoalNeighbors(areaLocation)) {
            if (areaMap[neighbor.first] != areaId) { 
                areaBorders[areaId].emplace(neighbor.first, neighbor.second);
            }
        }
    }
}

std::list<std::pair<int,int>> ReduceMap::getGoalNeighbors(int loc) const {
    std::list<std::pair<int,int>> neighbours;
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int i {0}; i < 4; ++i) {
        if (validateMove(candidates[i], loc)) {
            neighbours.emplace_back(std::make_pair(candidates[i], (i + 2) % 4));
        }
    }

    return neighbours;
}

std::list<std::pair<int,int>> ReduceMap::getNeighbors(int loc, int parent_loc) const {
    std::list<std::pair<int,int>> neighbors;
    int candidates[4] {loc + 1, loc + env->cols, loc - 1, loc - env->cols};

    for (int candidate : candidates) {
        if (candidate == parent_loc) { continue; }
        if (validateMove(candidate, loc)) {
            int cost;
            if (candidate == loc + (loc - parent_loc) || parent_loc == EMPTY) {
                cost = 1;
            } else {
                cost = 2;
            }

            neighbors.emplace_back(std::make_pair(candidate, cost));
        }
    }

    return neighbors;
}

bool ReduceMap::validateMove(int loc1, int loc2) const {
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

void ReduceMap::calculateDistanceBetweenAreas() {
    basePointDistances.resize(basePoints.size(), std::vector<std::pair<int,std::list<int>>>(basePoints.size(), {EMPTY, {}}));
    boost::asio::thread_pool pool(boost::thread::hardware_concurrency());

    for (int i {0}; i < basePoints.size(); ++i) {
        int start {basePoints[i]};
        boost::asio::post(pool, [this, start, i]() {
            dijkstra(start, i);
        });
    }

    pool.join();
}

void ReduceMap::dijkstra(int startLoc, int id) {
    std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNode::cmp> open;
    std::unordered_map<int, DijkstraNode*> closed;
    std::unordered_map<int, DijkstraNode*> allNodes;

    DijkstraNode* s = new DijkstraNode(startLoc, EMPTY, 0, nullptr);
    open.push(s);
    allNodes[s->location] = s;

    while (!open.empty()) {
        DijkstraNode* curr = open.top();
        open.pop();
        closed[curr->location] = curr;

        for (const auto& neighbor: getNeighbors(curr->location, curr->parentLocation)) {
            if (closed.find(neighbor.first) != closed.end()) {
                continue;
            }

            if (allNodes.find(neighbor.first) != allNodes.end()) {
                DijkstraNode* old = allNodes[neighbor.first];
                if (curr->g + neighbor.second < old->g) {
                    old->g = curr->g + neighbor.second;
                    old->parentLocation = curr->location;
                    old->parent = curr;
                }
            } 
            else {
                DijkstraNode* nextNode = new DijkstraNode(neighbor.first, curr->location, curr->g + neighbor.second, curr);
                open.push(nextNode);
                allNodes[nextNode->location] = nextNode;
            }
        }
    }

    for (int i {0}; i < basePoints.size(); ++i) {
        auto it {closed.find(basePoints[i])};
        if (it != closed.end()) {
            backTrack(id, i, it->second);
        }
    }

    for (auto& n : allNodes) {
        delete n.second;
    }
}

void ReduceMap::backTrack(int from, int to, DijkstraNode* current) {
    std::unordered_set<int> seen;
    std::list<int>& route {basePointDistances[from][to].second};
    route.push_front(to);
    //seen.insert(to);
    int areaId {to};

    // keep >>first<< or last occurence ??

    while (areaId != from) {
        current = current->parent;
        areaId = areaMap[current->location];

        if (/*seen.find(areaId) == seen.end() &&*/ route.front() != areaId) {
           route.push_front(areaId);
            // seen.insert(areaId);
        }
    }

    auto it {route.begin()};
    while (it != route.end()) {
        if (seen.find(*it) != seen.end()) {
            it = route.erase(it);
        } else {
            seen.insert(*it);
            ++it;
        }
    }
}