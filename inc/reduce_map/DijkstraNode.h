#ifndef DIJKSTRANODE_H
#define DIJKSTRANODE_H

class DijkstraNode {
public:
    int location;
    int parent_location;
    int g;

    DijkstraNode(int _location, int _parent_loc, int _g): location(_location), parent_location(_parent_loc), g(_g) { }

    struct cmp {
        bool operator() (const DijkstraNode* const a, const DijkstraNode* const b) const {
            return a->g > b->g;
        }
    };
};

#endif // DIJKSTRANODE_H