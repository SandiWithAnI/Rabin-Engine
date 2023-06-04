#pragma once
#include "Misc/PathfindingDetails.hpp"
struct AstarNode {
    float finalcost{}; // f(x)
    float givencost{}; // g(x)
    enum onList whichList;
    GridPos GridPosition;
    AstarNode* Parent;
};

enum onList {
    OPEN,
    WAITING,
    WALL,
    CLOSED
};

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

    float applyManhattan(Vec3& startpos, PathRequest& request);
    float applyChebyshev(Vec3& startpos, PathRequest& request);
    float applyEuclidean(Vec3& startpos, PathRequest& request);
    float applyOctile(Vec3& startpos, PathRequest& request);
    float applyInconsistent(Vec3& startpos, PathRequest& request);
   

    void setthemap();


    AstarNode**MapforAStar;
    std::list<AstarNode*>OpenList;

    bool once;

};


