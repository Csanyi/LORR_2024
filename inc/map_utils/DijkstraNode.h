#ifndef DIJKSTRANODE_H
#define DIJKSTRANODE_H

class DijkstraNode {
public:
    int location;
    int direction;
    int g;
    DijkstraNode* parent;

    DijkstraNode(int _location, int _direction, int _g, DijkstraNode* _parent = nullptr): 
        location(_location), direction(_direction), g(_g), parent(_parent) { }

    struct cmp {
        bool operator() (const DijkstraNode* const a, const DijkstraNode* const b) const {
            return a->g > b->g;
        }
    };
};

#endif // DIJKSTRANODE_H