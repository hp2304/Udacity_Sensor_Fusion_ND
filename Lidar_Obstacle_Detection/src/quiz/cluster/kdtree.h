/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <utility>
#include <random>
#include <list>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(): point({}), id(-1), left(nullptr), right(nullptr) {}

    Node(std::vector<float> arr, int setId)
    :	point(arr), id(setId), left(nullptr), right(nullptr)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

void printVec(std::vector<std::vector<float>> const &arr){
    std::cout << "\n";
    int i = 0;
    for(auto point: arr){
        std::cout << i << ": " << point[0] << ", " << point[1] << "\n";
        ++i;
    }
    std::cout << "\n";
}


int rearrange(std::vector<std::vector<float>> &arr, int pivot_idx, int start, int end, int dim){
    if(start == end){
        return start;
    }
    int swap_idx = start;
    float pivot_val = arr[pivot_idx][dim];

    std::swap(arr[pivot_idx], arr[end]);

    for(int i=start; i<=end; ++i){
        if(arr[i][dim] < pivot_val){
            std::swap(arr[i], arr[swap_idx]);
            ++swap_idx;
        }
    }
    std::swap(arr[swap_idx], arr[end]);
    return swap_idx;
}

int getRandomIdx(int min, int max){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(min, max);

    return distrib(gen);
}

void kthSmallest(std::vector<std::vector<float>> &arr, int k, int start, int end, int dim){
    if(start > end)
        return;

    int rand_idx = getRandomIdx(start, end);
    int pivot_idx = rearrange(arr, rand_idx, start, end, dim);
    int local_pivot_idx = pivot_idx - start;

    if(local_pivot_idx < k){
        kthSmallest(arr, k - (local_pivot_idx + 1), pivot_idx + 1, end, dim);
    }
    else if(local_pivot_idx > k){
        kthSmallest(arr, k, start, pivot_idx - 1, dim);
    }
}


void buildRecursive(std::vector<std::vector<float>> &points,
                    int start, int end,
                    Node* &parent,
                    bool is_left,
                    int const &nb_dims,
                    int depth = 0){
    // points format: x, y, id | x, y, z, id
    if(start > end)
        return;
    int med_idx = (start + end) / 2;

    kthSmallest(points, med_idx - start, start, end, depth % nb_dims);

    Node *node = new Node(points[med_idx], points[med_idx].back());
    if(is_left)
        parent->left = node;
    else
        parent->right = node;
    buildRecursive(points, med_idx + 1, end, node, false, nb_dims, depth + 1);
    buildRecursive(points, start, med_idx - 1, node, true, nb_dims, depth + 1);
}

float euclideanDist(std::vector<float> const &p1, std::vector<float> const &p2){
    int n = std::min(p1.size(), p2.size());
    float dist = 0;
    for(int i=0; i<n; ++i)
        dist += pow(p1[i] - p2[i], 2);
    return sqrt(dist);
}

struct KdTree
{
    Node* root;

    KdTree()
    : root(nullptr)
    {}

    KdTree(std::vector<std::vector<float>> const &points){
        auto points_copy = points;
        for(int i=0; i<points_copy.size(); ++i){
            points_copy[i].push_back((float) i);
        }
        int nb_dims = points[0].size();

        Node *fake_root = new Node();
        buildRecursive(points_copy, 0, points_copy.size() - 1, fake_root, true, nb_dims, 0);

        root = fake_root->left;
    }

    ~KdTree()
    {
        delete root;
    }

    void insert(std::vector<float> &point, int &id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root 
        if(root == nullptr){
            root = new Node(point, id);
            return;
        }

        Node *cur = root, *prev = nullptr;
        int depth = 0, nb_dims = root->point.size();
        bool is_left;
        while(cur != nullptr){
            int coord_idx = depth % nb_dims;
            prev = cur;
            if(cur->point[coord_idx] >= point[coord_idx]){
                cur = cur->left;
                is_left = true;
            }
            else{
                cur = cur->right;
                is_left = false;
            }
            ++depth;
        }
        if(is_left)
            prev->left = new Node(point, id);
        else
            prev->right = new Node(point, id);
        
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) const{
        std::vector<int> ids;
        int nb_dims = target.size();

        std::list<std::pair<Node *, int>> q;
        q.push_back(std::make_pair(root, 0));

        while(!q.empty()){
            auto cur = q.front();
            q.pop_front();

            Node *node = cur.first;
            int dim = cur.second;

            if(node == nullptr)
                continue;


            int next_dim = (dim + 1) % nb_dims;

            if(fabs(node->point[dim] - target[dim]) <= distanceTol) {
                float dist = euclideanDist(node->point, target);
                if(dist <= distanceTol)
                    ids.push_back(node->id);
            }

            if((target[dim] - distanceTol) < node->point[dim])
                q.push_back(std::make_pair(node->left, next_dim));
            if((target[dim] + distanceTol) > node->point[dim])
                q.push_back(std::make_pair(node->right, next_dim));
        }
        return ids;
    }
};


KdTree* buildTree(std::vector<std::vector<float>> const &points){
    auto points_copy = points;
    for(int i=0; i<points_copy.size(); ++i){
        points_copy[i].push_back((float) i);
    }
    int nb_dims = points[0].size();

    Node *fake_root = new Node();
    buildRecursive(points_copy, 0, points_copy.size() - 1, fake_root, true, nb_dims, 0);

    KdTree *tree = new KdTree;
    tree->root = fake_root->left;
    return tree;
}



