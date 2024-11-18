#ifndef DIJKSTRANODE_H
#define DIJKSTRANODE_H

class DijkstraNode {
public:
    int location;
    int parentLocation;
    int g;
    int nextArea;

    DijkstraNode(int _location, int _parentLoc, int _g, int _nextArea = -1): 
        location(_location), parentLocation(_parentLoc), g(_g), nextArea(_nextArea) { }

    struct cmp {
        bool operator() (const DijkstraNode* const a, const DijkstraNode* const b) const {
            return a->g > b->g;
        }
    };
};

#endif // DIJKSTRANODE_H