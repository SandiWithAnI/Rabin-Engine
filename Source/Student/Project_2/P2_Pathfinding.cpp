#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */
    Callback cb = std::bind(&AStarPather::setthemap, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);
  
    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
 /*   const int width = terrain->get_map_width();
    for (int i = 0; i < width; ++i) {
        delete[] MapforAStar;
    } 
    delete[] MapforAStar;*/
}

void AStarPather::setthemap() {
    //allocate memory for the map
    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();

    MapforAStar = new AstarNode * [width];
    for (int i = 0; i < width; ++i) {
        MapforAStar[i] = new AstarNode[height];
    } 

    //make all the nodes on the map on waiting n wall
    for (int j = 0; j < width;++j ) {
        for (int k = 0; k < width; ++k) {
            if (terrain->is_wall(j, k)) {
                MapforAStar[j][k].whichList = onList::WALL;

            }
            else {
            MapforAStar[j][k].whichList = onList::WAITING;
            MapforAStar[j][k].givencost = 0.0f;

            }
        }
    }
}

float AStarPather::applyManhattan(Vec3& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return (xdiff + ydiff);
}

float AStarPather::applyChebyshev(Vec3& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return std::max(xdiff, ydiff);
}

float AStarPather::applyEuclidean(Vec3& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return sqrt((xdiff * xdiff) + (ydiff * ydiff));
}

float AStarPather::applyOctile(Vec3& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return(std::min(xdiff, ydiff) * sqrt(2.0f) + std::max(xdiff, ydiff) - std::min(xdiff, ydiff));
}

float AStarPather::applyInconsistent(Vec3& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    if ((start.row + start.col) % 2 > 0) {
        return applyEuclidean(startpos, request);
    }

    return 0;
}



PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    // WRITE YOUR CODE HERE
   
    GridPos lowestcost;

    GridPos goal = terrain->get_grid_position(request.goal);

    if (request.newRequest) {
        float startheuristic = 0.0f;
        if (OpenList.empty()==false) {
            OpenList.clear();
        }
        if (request.settings.heuristic==Heuristic::CHEBYSHEV) {startheuristic = applyChebyshev(request.start, request); }
        if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(request.start,request);}
        if (request.settings.heuristic == Heuristic::INCONSISTENT) {startheuristic = applyInconsistent(request.start,request);}
        if (request.settings.heuristic == Heuristic::MANHATTAN) {startheuristic = applyManhattan(request.start,request);}
        if (request.settings.heuristic == Heuristic::OCTILE) {startheuristic = applyOctile(request.start,request);}

       
        GridPos getposition = terrain->get_grid_position(request.start);
        MapforAStar[getposition.row][getposition.col].finalcost = startheuristic + MapforAStar[getposition.row][getposition.col].givencost;
      
        //put the starting node on the openlist
        OpenList.push_back(&MapforAStar[getposition.row][getposition.col]);
        
    }

    GridPos start = terrain->get_grid_position(request.start);

    while (OpenList.empty()==false) {
 
       
        //pop the cheapest node on the list

        if (OpenList.size()==1) { //if only got one element in the list
            lowestcost= (*OpenList.begin())->GridPosition;
            terrain->set_color(lowestcost, Colors::Yellow);
            OpenList.pop_back();
        }
        else {
            std::list<AstarNode*>::iterator it = OpenList.begin();
            for (; std::next(it,1) != OpenList.end(); ++it) {
                if ((*it)->finalcost >  (*std::next(it,1))->finalcost ) {
                    lowestcost = (*std::next(it, 1))->GridPosition;
                }
                else {
                    lowestcost = (*it)->GridPosition;
                }
            }
            OpenList.erase(it);
            terrain->set_color(lowestcost, Colors::Yellow);
        }

        //if the node is goal node
        if (lowestcost== goal) {
            //push in the goal first
            request.path.push_front(request.goal);
            int getRow = goal.row;
            int getCol = goal.col;
            for (; MapforAStar[getRow][getCol].Parent->GridPosition != start;) {
                Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
                request.path.push_front(topushin);
                getRow = MapforAStar[getRow][getCol].Parent->GridPosition.row;
                getCol = MapforAStar[getRow][getCol].Parent->GridPosition.col;
            }
            return PathResult::COMPLETE;
        }

        MapforAStar[(lowestcost).row][(lowestcost).col].whichList = onList::CLOSED;
        
        const int width = terrain->get_map_width() -1;
        const int height = terrain->get_map_height() -1 ;

        //getting all valid neighbours
        if ((lowestcost).row !=0) { //checking left of parent
            float startheuristic = 0.0f;
            int pos = (lowestcost).row - 1;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][(lowestcost).col].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = lowestcost.col;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][(lowestcost).col].whichList = onList::OPEN;
                MapforAStar[pos][(lowestcost).col].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request);}
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) {startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) {startheuristic = applyManhattan(thestart, request);}
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request);}
                //link the parent
                MapforAStar[pos][lowestcost.col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][lowestcost.col].finalcost = MapforAStar[pos][(lowestcost).col].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][(lowestcost).col]);
            }
            //if its already on the open list
            if (MapforAStar[pos][(lowestcost).col].whichList == onList::OPEN) {
              //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) {startheuristic = applyChebyshev(thestart, request);}
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) {startheuristic = applyEuclidean(thestart, request);}
                if (request.settings.heuristic == Heuristic::INCONSISTENT) {startheuristic = applyInconsistent(thestart, request);}
                if (request.settings.heuristic == Heuristic::MANHATTAN) {startheuristic = applyManhattan(thestart, request);}
                if (request.settings.heuristic == Heuristic::OCTILE) {startheuristic = applyOctile(thestart, request);}
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it ) {
                    if ((*it)->GridPosition == MapforAStar[pos][(lowestcost).col].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][(lowestcost).col].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
            
        }
        if (lowestcost.row != width) { //checking right of parent
            float startheuristic = 0.0f;
            int pos = (lowestcost).row + 1;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][(lowestcost).col].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = lowestcost.col;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][(lowestcost).col].whichList = onList::OPEN;
                MapforAStar[pos][(lowestcost).col].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][lowestcost.col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][lowestcost.col].finalcost = MapforAStar[pos][(lowestcost).col].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][(lowestcost).col]);
            }
            //if its already on the open list
            if (MapforAStar[pos][(lowestcost).col].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][(lowestcost).col].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][(lowestcost).col].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }

        if (lowestcost.row != 0 && lowestcost.col != height) { //checking for top left
            float startheuristic = 0.0f;
            int ypos = (lowestcost).col - 1;
            int pos = (lowestcost).row + 1;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][ypos].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = ypos;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][ypos].whichList = onList::OPEN;
                MapforAStar[pos][ypos].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][ypos].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][ypos].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][ypos].finalcost = MapforAStar[pos][ypos].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][ypos]);
            }
            //if its already on the open list
            if (MapforAStar[pos][ypos].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][ypos].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][ypos].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }
        if (lowestcost.col != 0) { //checking for direct bottom
            float startheuristic = 0.0f;
            int ypos = (lowestcost).col - 1;
            int pos = (lowestcost).row ;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][ypos].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = ypos;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][ypos].whichList = onList::OPEN;
                MapforAStar[pos][ypos].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][ypos].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][ypos].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][ypos].finalcost = MapforAStar[pos][ypos].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][ypos]);
            }
            //if its already on the open list
            if (MapforAStar[pos][ypos].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][ypos].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][ypos].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }
        if (lowestcost.col != height) { //checking for top of parent
            float startheuristic = 0.0f;
            int ypos = (lowestcost).col + 1;
            int pos = (lowestcost).row;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][ypos].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = ypos;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][ypos].whichList = onList::OPEN;
                MapforAStar[pos][ypos].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][ypos].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][ypos].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][ypos].finalcost = MapforAStar[pos][ypos].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][ypos]);
            }
            //if its already on the open list
            if (MapforAStar[pos][ypos].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][ypos].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][ypos].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }

        if (lowestcost.row != width && lowestcost.col != 0) {//checking  bottom right
            float startheuristic = 0.0f;
            int ypos = (lowestcost).col - 1;
            int pos = (lowestcost).row + 1;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][ypos].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = ypos;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][ypos].whichList = onList::OPEN;
                MapforAStar[pos][ypos].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][ypos].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][ypos].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][ypos].finalcost = MapforAStar[pos][ypos].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][ypos]);
            }
            //if its already on the open list
            if (MapforAStar[pos][ypos].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][ypos].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][ypos].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }
        if (lowestcost.row != 0 && lowestcost.col != 0) { //checking bottom left
            float startheuristic = 0.0f;
            int ypos = (lowestcost).col - 1;
            int pos = (lowestcost).row -1 ;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][ypos].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = ypos;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][ypos].whichList = onList::OPEN;
                MapforAStar[pos][ypos].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][ypos].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][ypos].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][ypos].finalcost = MapforAStar[pos][ypos].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][ypos]);
            }
            //if its already on the open list
            if (MapforAStar[pos][ypos].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][ypos].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][ypos].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }
        if (lowestcost.row != width && lowestcost.col != height) { //checking top right
            float startheuristic = 0.0f;
            int ypos = (lowestcost).col + 1;
            int pos = (lowestcost).row + 1;
            //ensure its not a wall, if not a wall then put on open list
            if (MapforAStar[pos][ypos].whichList == onList::WAITING) {
                GridPos colorchange;
                colorchange.row = pos;
                colorchange.col = ypos;
                terrain->set_color(colorchange, Colors::Blue);
                MapforAStar[pos][ypos].whichList = onList::OPEN;
                MapforAStar[pos][ypos].givencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][ypos].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                //link the parent
                MapforAStar[pos][ypos].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                //calculate final cost
                MapforAStar[pos][ypos].finalcost = MapforAStar[pos][ypos].givencost + startheuristic;
                OpenList.push_back(&MapforAStar[pos][ypos]);
            }
            //if its already on the open list
            if (MapforAStar[pos][ypos].whichList == onList::OPEN) {
                //heuristics will not change only the given cost will change
                Vec3 thestart = terrain->get_world_position(MapforAStar[pos][(lowestcost).col].GridPosition);
                if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
                if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
                if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
                if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
                if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
                float newgivencost = ++MapforAStar[(lowestcost).row][(lowestcost).col].givencost;
                float newfinalcost = startheuristic + newgivencost;

                for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end(); ++it) {
                    if ((*it)->GridPosition == MapforAStar[pos][ypos].GridPosition) {
                        //update the values if its cheaper
                        if (newfinalcost < MapforAStar[pos][ypos].finalcost) {
                            (*it)->finalcost = newfinalcost;
                            (*it)->givencost = newgivencost;
                            (*it)->Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }


        if (request.settings.singleStep == true) {
            return PathResult::PROCESSING;
        }

    }

   
    return PathResult::IMPOSSIBLE;
    
    //// Just sample code, safe to delete
   /* GridPos start = terrain->get_grid_position(request.start);
    GridPos goal = terrain->get_grid_position(request.goal);
    terrain->set_color(start, Colors::Orange);
    terrain->set_color(goal, Colors::Orange);
    request.path.push_back(request.start);
    request.path.push_back(request.goal);*/


  /*  request.path.push_back(request.start);
    GridPos pos1;
    pos1.col = 1;
    pos1.row = 15;

    GridPos pos2;
    pos2.col = 2;
    pos2.row = 10;
    Vec3 one = terrain->get_world_position(pos1);
    Vec3 two = terrain->get_world_position(pos2);

    request.path.push_back(two);
    request.path.push_back(one);
    request.path.push_back(request.goal);*/

}
