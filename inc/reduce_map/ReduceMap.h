#ifndef REDUCEMAP_H
#define REDUCEMAP_H

#include <vector>
#include <list>
#include <unordered_map>
#include "SharedEnv.h"

class ReduceMap {
public:
    ReduceMap(SharedEnvironment* _env): env(_env) { }

    void reduceMapStart();
    bool reduceMap(bool keepSalient);

    void reduceMapWaypointsStart();
    bool reduceReduceMapWaypoints(int distance);

    void printReducedMap() const;
    void printReducedMapWaypoints() const;

    void hierarchyMapStart(int grid);

    void divideIntoAreas(int distance);
    void calculateDistanceBetweenAreas();

    void eraseDeadEnds();

    std::vector<int> deadEndMap;
    std::vector<int> basePointMap;

private:
    const int MAP_PRINT_WIDTH = 550;
    const int MAP_PRINT_OFFSET = 0;

    SharedEnvironment* env;
    std::vector<int> reducedMap;
    std::vector<std::unordered_map<int,int>> reducedMapWaypoints;

    std::vector<int> basePoints;

    void reduceMapUpdate();

    bool areNeighboursTraversable(int location) const;;
    bool isNorthEastEmpty(int location, int x, int y) const;
    bool isSouthEastEmpty(int location, int x, int y) const;
    bool isNorthWestEmpty(int location, int x, int y) const;
    bool isSouthWestEmpty(int location, int x, int y) const;

    int countReducedVerticalNeighbours(int location) const;
    int countReducedHorizontalNeighbours(int location) const;

    bool isDeadLoc(int location) const;

    void markBasePoints(int distance);
    bool markRandomPoint(int row, int col, int distance, int id);
    void createAreasAroundBasePoints();

    std::list<std::pair<int,int>> getNeighbors(int loc, int parent_loc) const;
    bool validateMove(int loc1, int loc2) const;
};

#endif // REDUCEMAP_H
