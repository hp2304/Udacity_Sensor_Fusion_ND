#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <utility>
#include <random>
#include <list>
#include <unordered_set>
#include <iostream>
#include <vector>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(): point({}), id(-1), left(nullptr), right(nullptr) {}

    Node(std::vector<float> &arr, int setId)
    :	point(arr), id(setId), left(nullptr), right(nullptr)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

void printVec(std::vector<std::vector<float>> const &arr);

int rearrange(std::vector<std::vector<float>> &arr, int pivot_idx, int start, int end, int dim);

std::vector<int> sampleRandomIndices(int min, int max, int nb_samples);


void kthSmallest(std::vector<std::vector<float>> &arr, int k, int start, int end, int dim);

void buildRecursive(std::vector<std::vector<float>> &points,
                    int start, int end,
                    Node* &parent,
                    bool is_left,
                    int const &nb_dims,
                    int depth = 0);

float euclideanDist(std::vector<float> const &p1, std::vector<float> const &p2);

struct KdTree
{
    Node* root;

    KdTree()
    : root(nullptr)
    {}

    // TODO: Use pass by reference and update relevant code for correctness of clustering algorithm
    KdTree(std::vector<std::vector<float>> &points);

    ~KdTree()
    {
        delete root;
    }

    void insert(std::vector<float> &point, int &id);

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) const;
};


void explore(std::unordered_set<int> &visited,
             KdTree* &tree,
             const std::vector<std::vector<float>>& points,
             int point_id,
             std::vector<int> &cluster,
             float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                                KdTree* &tree,
                                                float distanceTol,
                                                int minSize,
                                                int maxSize);

#endif



