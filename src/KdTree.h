//
// Created by Jana on 02.11.2023.
//

#ifndef LASCAMPUS_KDTREE_H
#define LASCAMPUS_KDTREE_H


#include <vector>
#include <queue>
#include "Vertex.h"

struct KdTreeNode {

    Vertex vertex;
    int index;
    KdTreeNode* left;
    KdTreeNode* right;

    int cutDim;
    Vertex lobound, upbound;

};
class kNNSearchElem {
public:
    int dataIndex;  // index of actual kdnode in *allnodes*
    double distance;   // distance of this neighbor from *point*
    kNNSearchElem(int i, double d) {
        dataIndex = i;
        distance = d;
    }
};
class kNNSearchComparator {
public:
    bool operator()(const kNNSearchElem& n, const kNNSearchElem& m) {
        return (n.distance < m.distance);
    }
};

typedef std::priority_queue<kNNSearchElem, std::vector<kNNSearchElem>, kNNSearchComparator> SearchQueue;

class KdTree {
public:
    KdTree(const std::vector<KdTreeNode>* nodes);
    void k_nearest_neighbors(const Vertex& point, size_t k,
                             std::vector<KdTreeNode>* result);

private:
    // helper variable for keeping track of subtree bounding box
    Vertex lobound, upbound;
    KdTreeNode* build_tree(size_t depth, size_t a, size_t b);
    std::vector<KdTreeNode> nodes;
    KdTreeNode* root;
    bool neighbor_search(const Vertex& point, KdTreeNode* node, size_t k, SearchQueue* neighborheap);
    bool bounds_overlap_ball(const Vertex& point, double dist, KdTreeNode* node);
    bool ball_within_bounds(const Vertex& point, double dist, KdTreeNode* node);


};

inline double distance(const Vertex& point1, const Vertex& point2);
inline double coordinate_distance(float x, float y, size_t dim);




#endif //LASCAMPUS_KDTREE_H
