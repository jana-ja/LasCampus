//
// Created by Jana on 02.11.2023.
//

#include <stdexcept>
#include "KdTree.h"

// TODO interface schreiben mit dingen die ich von meiner datenstruktur erwarte???
class compare_dimension {
public:
    compare_dimension(size_t dim) { d = dim; }
    bool operator()(const KdTreeNode& p, const KdTreeNode& q) {
        return (p.vertex[d] < q.vertex[d]);
    }
    size_t d;
};

KdTree::KdTree(const std::vector<KdTreeNode>* pNodes) {
    size_t i, j;
    double val;
    // copy over input data
    if (!pNodes || pNodes->empty())
        throw std::invalid_argument("kdtree::KdTree(): argument pNodes must not be empty");
    nodes = *pNodes;
    // compute global bounding box
    lobound = pNodes->begin()->vertex;
    upbound = pNodes->begin()->vertex;
    for (i = 1; i < pNodes->size(); i++) {
        for (j = 0; j < 3; j++) {
            val = pNodes[i].vertex[j];
            if (lobound[j] > val) lobound[j] = val;
            if (upbound[j] < val) upbound[j] = val;
        }
    }
    // build tree recursively
    root = build_tree(0, 0, pNodes.size());
}

//--------------------------------------------------------------
// recursive build of tree
// "a" and "b"-1 are the lower and upper indices
// from "allnodes" from which the subtree is to be built
//--------------------------------------------------------------
KdTreeNode* KdTree::build_tree(size_t depth, size_t a, size_t b) {
    size_t m;
    double temp, cutval;
    KdTreeNode* node = new KdTreeNode();
    node->lobound = lobound;
    node->upbound = upbound;
    node->cutDim = depth % 3;
    if (b - a <= 1) {
        node->index = a;
        node->vertex = nodes[a].vertex;
    } else {
        m = (a + b) / 2;
        std::nth_element(nodes.begin() + a, nodes.begin() + m,
                         nodes.begin() + b, compare_dimension(node->cutDim));
        node->vertex = nodes[m].vertex;
        cutval = nodes[m].vertex[node->cutDim];
        node->index = m;
        if (m - a > 0) {
            temp = upbound[node->cutDim];
            upbound[node->cutDim] = cutval;
            node->left = build_tree(depth + 1, a, m);
            upbound[node->cutDim] = temp;
        }
        if (b - m > 1) {
            temp = lobound[node->cutdim];
            lobound[node->cutdim] = cutval;
            node->right = build_tree(depth + 1, m + 1, b);
            lobound[node->cutdim] = temp;
        }
    }
    return node;
}

void KdTree::k_nearest_neighbors(const Vertex& point, size_t k,
                                 std::vector<KdTreeNode>* result) {
    size_t i;
    KdTreeNode temp;

    result->clear();
    if (k < 1) return;

    // collect result of k values in neighborheap
    //std::priority_queue<kNNSearchElem, std::vector<kNNSearchElem>, kNNSearchComparator>*
    //neighborheap = new std::priority_queue<kNNSearchElem, std::vector<kNNSearchElem>, kNNSearchComparator>();
    SearchQueue* neighborheap = new SearchQueue();
    if (k > nodes.size()) {
        // when more neighbors asked than nodes in tree, return everything
        k = nodes.size();
        for (i = 0; i < k; i++) {
                neighborheap->push(
                        kNNSearchElem(i, distance(nodes[i].vertex, point)));
        }
    } else {
        neighbor_search(point, root, k, neighborheap);
    }

    // copy over result sorted by distance
    // (we must revert the vector for ascending order)
    while (!neighborheap->empty()) {
        i = neighborheap->top().dataIndex;
        neighborheap->pop();
        result->push_back(nodes[i]);
    }
    // beware that less than k results might have been returned
    k = result->size();
    for (i = 0; i < k / 2; i++) {
        temp = (*result)[i];
        (*result)[i] = (*result)[k - 1 - i];
        (*result)[k - 1 - i] = temp;
    }
    delete neighborheap;
}

//--------------------------------------------------------------
// recursive function for nearest neighbor search in subtree
// under *node*. Stores result in *neighborheap*.
// returns "true" when no nearer neighbor elsewhere possible
//--------------------------------------------------------------
bool KdTree::neighbor_search(const Vertex& point, KdTreeNode* node,
                             size_t k, SearchQueue* neighborheap) {
    double curdist, dist;

    curdist = distance(point, node->vertex);

        if (neighborheap->size() < k) {
            neighborheap->push(kNNSearchElem(node->index, curdist));
        } else if (curdist < neighborheap->top().distance) {
            neighborheap->pop();
            neighborheap->push(kNNSearchElem(node->index, curdist));
        }

    // first search on side closer to point
    if (point[node->cutDim] < node->vertex[node->cutDim]) {
        if (node->left)
            if (neighbor_search(point, node->left, k, neighborheap)) return true;
    } else {
        if (node->right)
            if (neighbor_search(point, node->right, k, neighborheap)) return true;
    }
    // second search on farther side, if necessary
    if (neighborheap->size() < k) {
        dist = std::numeric_limits<double>::max();
    } else {
        dist = neighborheap->top().distance;
    }
    if (point[node->cutDim] < node->vertex[node->cutDim]) {
        if (node->left && bounds_overlap_ball(point, dist, node->right))
            if (neighbor_search(point, node->right, k, neighborheap)) return true;
    } else {
        if (node->right && bounds_overlap_ball(point, dist, node->left))
            if (neighbor_search(point, node->left, k, neighborheap)) return true;
    }

    if (neighborheap->size() == k) dist = neighborheap->top().distance;
    return ball_within_bounds(point, dist, node);
}

// returns true when the bounds of *node* overlap with the
// ball with radius *distance* around *point*
bool KdTree::bounds_overlap_ball(const Vertex& point, double dist,
                                 KdTreeNode* node) {
    double distsum = 0.0;
    size_t i;
    for (i = 0; i < 3; i++) {
        if (point[i] < node->lobound[i]) {  // lower than low boundary
            distsum += coordinate_distance(point[i], node->lobound[i]);
            if (distsum > dist) return false;
        } else if (point[i] > node->upbound[i]) {  // higher than high boundary
            distsum += coordinate_distance(point[i], node->upbound[i]);
            if (distsum > dist) return false;
        }
    }
    return true;
}

// returns true when the bounds of *node* completely contain the
// ball with radius *distance* around *point*
bool KdTree::ball_within_bounds(const Vertex& point, double dist,
                                KdTreeNode* node) {
    size_t i;
    for (i = 0; i < 3; i++)
        if (coordinate_distance(point[i], node->lobound[i], i) <= dist ||
            coordinate_distance(point[i], node->upbound[i], i) <= dist)
            return false;
    return true;
}

double coordinate_distance(double x, double y) {
        return (x - y) * (x - y);
}