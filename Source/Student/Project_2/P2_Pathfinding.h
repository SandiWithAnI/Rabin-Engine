#pragma once
#include "Misc/PathfindingDetails.hpp"

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

    float applyManhattan(PathRequest& request);
    float applyChebyshev(PathRequest& request);
    float applyEuclidean(PathRequest& request);
    float applyOctile(PathRequest& request);

    struct Node {
        Node* Parent;
        GridPos GridPosition;
        float finalcost{}; // f(x)
        float givencost{}; // g(x)
        enum onList whichList;
    };

    enum onList {
        OPEN,
        CLOSED
    };

    std::list<Node>OpenList;
};

