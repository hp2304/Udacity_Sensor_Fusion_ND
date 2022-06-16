#include "kdtree.h"
#include <vector>
#include <iostream>


int main(){
    std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.9,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };

    KdTree* tree = new KdTree(points);
    // tree->root = buildTree(points);

    std::cout << "Test Search" << std::endl;
    std::vector<int> nearby = tree->search({-6,7},3.0);
    for(int index : nearby)
        std::cout << index << ",";
    std::cout << std::endl;

    return 0;
}