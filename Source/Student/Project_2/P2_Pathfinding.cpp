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

}

void AStarPather::setthemap() {
    //allocate memory for the map
    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();


    //make all the nodes on the map on waiting n wall
    for (int j = 0; j < width;++j ) {
        for (int k = 0; k < height; ++k) {
            if (terrain->is_wall(j, k)) {
                MapforAStar[j][k].whichList = onList::WALL;
                MapforAStar[j][k].givencost = 0.0f;
                MapforAStar[j][k].GridPosition.row = j;
                MapforAStar[j][k].GridPosition.col = k;
            }
            else {
            MapforAStar[j][k].whichList = onList::WAITING;
            MapforAStar[j][k].givencost = 0.0f;
            MapforAStar[j][k].GridPosition.row = j;
            MapforAStar[j][k].GridPosition.col = k;

            }
        }
    }
}

float AStarPather::applyManhattan(Vec3 const& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return (xdiff + ydiff);
}

float AStarPather::applyChebyshev(Vec3 const& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return std::max(xdiff, ydiff);
}

float AStarPather::applyEuclidean(Vec3 const& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return sqrt((xdiff * xdiff) + (ydiff * ydiff));
}

float AStarPather::applyOctile(Vec3 const& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);

    float xdiff = abs(static_cast<float>(start.row - goal.row));
    float ydiff = abs(static_cast<float>(start.col - goal.col));

    return(std::min(xdiff, ydiff) * sqrt(2.0f) + std::max(xdiff, ydiff) - std::min(xdiff, ydiff));
}

float AStarPather::applyInconsistent(Vec3 const& startpos, PathRequest& request) {
    GridPos start = terrain->get_grid_position(startpos);
    GridPos goal = terrain->get_grid_position(request.goal);


    if ((start.row + start.col) % 2 > 0 ) {
        return applyEuclidean(startpos, request);
    }

    return 0;
}
void AStarPather::checkingdiagonals(int row, int col, PathRequest& request, GridPos lowestcost, float startheuristic) {
  
    if (MapforAStar[row][col].whichList == onList::WAITING) {
        GridPos colorchange;
        colorchange.row = row;
        colorchange.col = col;
        if (request.settings.debugColoring) {
            terrain->set_color(colorchange, Colors::Blue);
        }
        MapforAStar[row][col].whichList = onList::OPEN;
        MapforAStar[row][col].givencost = MapforAStar[(lowestcost).row][(lowestcost).col].givencost + sqrt(2.0f);
        Vec3 thestart = terrain->get_world_position(MapforAStar[row][col].GridPosition);
        if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
        if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
        if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
        if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
        if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
        //link the parent
        MapforAStar[row][col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
        //calculate final cost
        MapforAStar[row][col].finalcost = MapforAStar[row][col].givencost + (startheuristic * request.settings.weight);
        OpenList.push_back(&MapforAStar[row][col]);
    }
    //if its already on the open list
    else if (MapforAStar[row][col].whichList == onList::OPEN || MapforAStar[row][col].whichList == onList::CLOSED) {
        //heuristics will not change only the given cost will change
        Vec3 thestart = terrain->get_world_position(MapforAStar[row][col].GridPosition);
        if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
        if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
        if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
        if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
        if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
        float newgivencost = MapforAStar[(lowestcost).row][(lowestcost).col].givencost + sqrt(2.0f);
        float newfinalcost = (startheuristic * request.settings.weight) + newgivencost;

        if (MapforAStar[row][col].whichList == onList::OPEN) {
           

            if (MapforAStar[row][col].givencost > newgivencost) { //if its cheaper than original, put it on the openlist
                //update all the values
              
                MapforAStar[row][col].givencost = newgivencost;
                MapforAStar[row][col].finalcost = newfinalcost;
                MapforAStar[row][col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
               
            }
        }
        else if (MapforAStar[row][col].whichList == onList::CLOSED) { //if it was on the closed list
            if (MapforAStar[row][col].givencost > newgivencost) { //if its cheaper than original, put it on the openlist
                //update all the values
                MapforAStar[row][col].whichList = onList::OPEN;
                MapforAStar[row][col].givencost = newgivencost;
                MapforAStar[row][col].finalcost = newfinalcost;
                MapforAStar[row][col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                if (request.settings.debugColoring) {
                    GridPos colorchange;
                    colorchange.row = row;
                    colorchange.col = col;
                    terrain->set_color(colorchange, Colors::Blue);
                }
                OpenList.push_back(&MapforAStar[row][col]);
            }

        }
    }
}
void AStarPather::checkingneighbours(int row, int col, PathRequest& request, GridPos lowestcost, float startheuristic) {
 
    //ensure its not a wall, if not a wall then put on open list
    if (MapforAStar[row][col].whichList == onList::WAITING) {
        GridPos colorchange;
        colorchange.row = row;
        colorchange.col = col;
        if (request.settings.debugColoring) {
            terrain->set_color(colorchange, Colors::Blue);
        }
        MapforAStar[row][col].whichList = onList::OPEN;
        MapforAStar[row][col].givencost = MapforAStar[(lowestcost).row][(lowestcost).col].givencost + 1.0f;
        Vec3 thestart = terrain->get_world_position(MapforAStar[row][col].GridPosition);
        if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
        if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
        if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
        if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
        if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
        //link the parent
        MapforAStar[row][col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
        //calculate final cost
        MapforAStar[row][col].finalcost = MapforAStar[row][col].givencost + (startheuristic * request.settings.weight);
        OpenList.push_back(&MapforAStar[row][col]);
    }
    //if its already on the open list
    else if (MapforAStar[row][col].whichList == onList::OPEN || MapforAStar[row][col].whichList == onList::CLOSED) {
        //heuristics will not change only the given cost will change
        Vec3 thestart = terrain->get_world_position(MapforAStar[row][col].GridPosition);
        if (request.settings.heuristic == Heuristic::OCTILE) { startheuristic = applyOctile(thestart, request); }
        if (request.settings.heuristic == Heuristic::CHEBYSHEV) { startheuristic = applyChebyshev(thestart, request); }
        if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(thestart, request); }
        if (request.settings.heuristic == Heuristic::INCONSISTENT) { startheuristic = applyInconsistent(thestart, request); }
        if (request.settings.heuristic == Heuristic::MANHATTAN) { startheuristic = applyManhattan(thestart, request); }
        float newgivencost = MapforAStar[(lowestcost).row][(lowestcost).col].givencost + 1.0f;
        float newfinalcost = (startheuristic * request.settings.weight) + newgivencost;

        if (MapforAStar[row][col].whichList == onList::OPEN) {
           
            if (MapforAStar[row][col].givencost > newgivencost) { //if its cheaper than original, put it on the openlist
               //update all the values

                MapforAStar[row][col].givencost = newgivencost;
                MapforAStar[row][col].finalcost = newfinalcost;
                MapforAStar[row][col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];

            }
        }
        else if (MapforAStar[row][col].whichList == onList::CLOSED) { //if it was on the closed list
            if (MapforAStar[row][col].givencost > newgivencost) { //if its cheaper than original, put it on the openlist
                //update all the values
                MapforAStar[row][col].whichList = onList::OPEN;
                MapforAStar[row][col].givencost = newgivencost;
                MapforAStar[row][col].finalcost = newfinalcost;
                MapforAStar[row][col].Parent = &MapforAStar[(lowestcost).row][(lowestcost).col];
                if (request.settings.debugColoring) {
                    GridPos colorchange;
                    colorchange.row = row;
                    colorchange.col = col;
                    terrain->set_color(colorchange, Colors::Blue);
                }

                OpenList.push_back(&MapforAStar[row][col]);
            }

        }
    }
}

void AStarPather::catmull(WaypointList& edittinglist, int getRow, int getCol, PathRequest& request, bool needtosort) {
    if (needtosort == true) {
        int temp = 0;
        GridPos start = terrain->get_grid_position(request.start);

        if (MapforAStar[getRow][getCol].GridPosition == MapforAStar[start.row][start.col].GridPosition) {
            Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
            edittinglist.push_front(topushin);
            edittinglist.push_front(request.start);
            return;
        }


        for (; MapforAStar[getRow][getCol].Parent != &MapforAStar[start.row][start.col];) {

            Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
            edittinglist.push_front(topushin);
            temp = getRow;
            getRow = MapforAStar[getRow][getCol].Parent->GridPosition.row;
            getCol = MapforAStar[temp][getCol].Parent->GridPosition.col;
        }
        Vec3 topushin2 = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
        edittinglist.push_front(topushin2);
        edittinglist.push_front(request.start);
    }

    //beginning node
    std::list<Vec3>::iterator startiterator = edittinglist.begin();
    //2nd node, points to the same first one
    std::list<Vec3>::iterator seconditerator = edittinglist.begin();
    //3rd node
    std::list<Vec3>::iterator thirditerator = std::next(edittinglist.begin()) ;
    //4th node
    std::list<Vec3>::iterator lastiterator = std::next(std::next(edittinglist.begin()));

    for (; lastiterator != edittinglist.end();) {
        Vec3 topushin = Vec3::CatmullRom(*startiterator, *seconditerator, *thirditerator, *lastiterator, 0.25);
        Vec3 topushin2 = Vec3::CatmullRom(*startiterator, *seconditerator, *thirditerator, *lastiterator, 0.50);
        Vec3 topushin3 = Vec3::CatmullRom(*startiterator, *seconditerator, *thirditerator, *lastiterator, 0.75);
        edittinglist.insert(thirditerator, topushin);
        edittinglist.insert(thirditerator, topushin2);
        edittinglist.insert(thirditerator, topushin3);
        startiterator = seconditerator;
            seconditerator = thirditerator;
            thirditerator = lastiterator;
            ++lastiterator;
    }

    Vec3 the025 = Vec3::CatmullRom(*startiterator, *seconditerator, *thirditerator, *thirditerator, 0.25);
    Vec3 the050 = Vec3::CatmullRom(*startiterator, *seconditerator, *thirditerator, *thirditerator, 0.50);
    Vec3 the075 = Vec3::CatmullRom(*startiterator, *seconditerator, *thirditerator, *thirditerator , 0.75);
    edittinglist.insert(thirditerator, the025);
    edittinglist.insert(thirditerator, the050);
    edittinglist.insert(thirditerator, the075);
    
  
    request.path = edittinglist;
    return;

}

void AStarPather::rubberbanding(WaypointList& edittinglist, int getRow, int getCol, PathRequest& request) {
    int temp = 0;
    GridPos start = terrain->get_grid_position(request.start);

    if (MapforAStar[getRow][getCol].GridPosition == MapforAStar[start.row][start.col].GridPosition) {
        Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
        edittinglist.push_front(topushin);
        edittinglist.push_front(request.start);
        return;
    }


    for (; MapforAStar[getRow][getCol].Parent != &MapforAStar[start.row][start.col];) {

        Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
        edittinglist.push_front(topushin);
        temp = getRow;
        getRow = MapforAStar[getRow][getCol].Parent->GridPosition.row;
        getCol = MapforAStar[temp][getCol].Parent->GridPosition.col;
    }
    Vec3 topushin2 = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
    edittinglist.push_front(topushin2);
    edittinglist.push_front(request.start);

    //beginning node
    std::list<Vec3>::iterator startiterator = edittinglist.begin();
    //middle node
    std::list<Vec3>::iterator middleiterator = std::next(edittinglist.begin());
    //3rd node
    std::list<Vec3>::iterator lastiterator = std::next(std::next(edittinglist.begin()));


    for (; lastiterator != edittinglist.end();) {

        GridPos startgrid = terrain->get_grid_position(*startiterator);
        GridPos endgrid = terrain->get_grid_position(*lastiterator);
        int rowMin = std::min(startgrid.row, endgrid.row);
        int rowMax = std::max(startgrid.row, endgrid.row);

        int colMin = std::min(startgrid.col, endgrid.col);
        int colMax = std::max(startgrid.col, endgrid.col);

        bool thereiswall = false;

        for (int i = rowMin; i <= rowMax; ++i) {
            for (int j = colMin; j <= colMax; ++j) {
                if (terrain->is_wall(i, j)) {
                    thereiswall = true;
                    break;
                }
            }
        }

        if (thereiswall == true) {
            ++startiterator;
            ++middleiterator;
            ++lastiterator;
        }
        else { //there is no wall
            //erase the middle grid
            edittinglist.erase(middleiterator);
            middleiterator = lastiterator;
            ++lastiterator;
        }
    }
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
   // bool diagonalcalculation = true;
    bool thereiswall = false;
    GridPos lowestcost;

    GridPos goal = terrain->get_grid_position(request.goal);

    if (request.newRequest) {
       
        float startheuristic = 0.0f;
        const int width = terrain->get_map_width();
        const int height = terrain->get_map_height();
        //reset 
        
            for (int j = 0; j < width; ++j) {
                for (int k = 0; k < height; ++k) {
                    if (terrain->is_wall(j, k)) {
                        MapforAStar[j][k].whichList = onList::WALL;
                        MapforAStar[j][k].givencost = 0.0f;
                        MapforAStar[j][k].GridPosition.row = j;
                        MapforAStar[j][k].GridPosition.col = k;
                    }
                    else {
                        MapforAStar[j][k].whichList = onList::WAITING;
                        MapforAStar[j][k].givencost = 0.0f;
                        MapforAStar[j][k].finalcost = 0.0f;
                        MapforAStar[j][k].GridPosition.row = j;
                        MapforAStar[j][k].GridPosition.col = k;
                        MapforAStar[j][k].Parent = nullptr;

                    }
                }
            }
            OpenList.clear();
        
        if (request.settings.heuristic == Heuristic::OCTILE) {startheuristic = applyOctile(request.start,request);}
        if (request.settings.heuristic==Heuristic::CHEBYSHEV) {startheuristic = applyChebyshev(request.start, request); }
        if (request.settings.heuristic == Heuristic::EUCLIDEAN) { startheuristic = applyEuclidean(request.start,request);}
        if (request.settings.heuristic == Heuristic::INCONSISTENT) {startheuristic = applyInconsistent(request.start,request);}
        if (request.settings.heuristic == Heuristic::MANHATTAN) {startheuristic = applyManhattan(request.start,request);}

       
        GridPos getposition = terrain->get_grid_position(request.start);
        MapforAStar[getposition.row][getposition.col].finalcost = (startheuristic * request.settings.weight) + 0;
      
        //put the starting node on the openlist
        OpenList.push_back(&MapforAStar[getposition.row][getposition.col]);
        
    }

    GridPos start = terrain->get_grid_position(request.start);

    while (OpenList.empty()==false) {



        //pop the cheapest node on the list

        if (OpenList.size()==1) { //if only got one element in the list
            lowestcost= (*OpenList.begin())->GridPosition;
            if (request.settings.debugColoring) {
                terrain->set_color(lowestcost, Colors::Yellow);
            }

            OpenList.pop_back();
        }
        else {
            lowestcost = OpenList.front()->GridPosition;

            std::list<AstarNode*>::iterator toerase = OpenList.begin();

            for (std::list<AstarNode*>::iterator it = OpenList.begin(); it != OpenList.end();++it) {
                if ((*it)->finalcost < (*toerase)->finalcost) {
                    toerase = it;
                }
            }
            lowestcost = (*toerase)->GridPosition;                  

            OpenList.erase(toerase);
            if (request.settings.debugColoring) {
                terrain->set_color(lowestcost, Colors::Yellow);
            }
        }

       
        MapforAStar[(lowestcost).row][(lowestcost).col].whichList = onList::CLOSED;

        //if the node is goal node
        if (lowestcost== goal) {
            int getRow = goal.row;
            int getCol = goal.col;   
            int temp = 0;
            
            //if the rubberbanding setting is on and smoothing not on
            if (request.settings.rubberBanding == true && request.settings.smoothing==false) {
              
                WaypointList edittinglist{}; //std::list<Vec3>
                rubberbanding(edittinglist, getRow, getCol, request);
                request.path = edittinglist;
                return PathResult::COMPLETE;

            }

            //if the smoothing setting is on and the rubberbanding is not on
            if (request.settings.smoothing == true && request.settings.rubberBanding == false) {
                WaypointList edittinglist{}; //std::list<Vec3>
                catmull(edittinglist,getRow,getCol,request,true);
                return PathResult::COMPLETE;                
            }

            
            //if both rubberbanding and catmull is on
            if (request.settings.smoothing == true && request.settings.rubberBanding == true) {
                WaypointList edittinglist{}; //std::list<Vec3>
                //do rubberbanding first
                rubberbanding(edittinglist, getRow, getCol, request);

                std::list<Vec3>::iterator startiterator = edittinglist.begin();
                std::list<Vec3>::iterator seconditerator = std::next (edittinglist.begin());
                while (seconditerator != edittinglist.end()) {

                    GridPos startgrid = terrain->get_grid_position(*startiterator);
                    GridPos endgrid = terrain->get_grid_position(*seconditerator);
                    int rowMin = std::min(startgrid.row, endgrid.row);
                    int rowMax = std::max(startgrid.row, endgrid.row);

                    int colMin = std::min(startgrid.col, endgrid.col);
                    int colMax = std::max(startgrid.col, endgrid.col);

                    if (static_cast<float> (abs((colMax - colMin))) >= 1.5f || static_cast<float> (abs((rowMax - rowMin))) >= 1.5f) {
                        Vec3 midpoint = (0.5 * (*seconditerator - *startiterator)) + *startiterator;
                        //insert the midpoint inside the list
                        edittinglist.insert(seconditerator, midpoint);
                        //make the midpoint the second iterator
                        --seconditerator;
                    }
                    else {
                        ++startiterator;
                        ++seconditerator;
                    }
                }

                catmull(edittinglist, getRow, getCol, request, false);
                return PathResult::COMPLETE;
            }


            //if the end position is the start position
            if (MapforAStar[getRow][getCol].GridPosition == MapforAStar[start.row][start.col].GridPosition) {
                Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
                request.path.push_front(topushin);
                request.path.push_front(request.start);
                return PathResult::COMPLETE;
            }
            else { // the end position is not the start position

                              
                for (; MapforAStar[getRow][getCol].Parent != &MapforAStar[start.row][start.col];) {
                   
                    Vec3 topushin = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
                    request.path.push_front(topushin);
                    temp = getRow;
                    getRow = MapforAStar[getRow][getCol].Parent->GridPosition.row;
                    getCol = MapforAStar[temp][getCol].Parent->GridPosition.col;
                }
                Vec3 topushin2 = terrain->get_world_position(MapforAStar[getRow][getCol].GridPosition);
                request.path.push_front(topushin2);
               
                request.path.push_front(request.start);
               
                return PathResult::COMPLETE;
            }
            return PathResult::COMPLETE;
        }
       
        
        const int width = terrain->get_map_width() -1;
        const int height = terrain->get_map_height() -1 ;

/********************************************************CALCULATION****************************************************************************************/

      
            //bottom
            if ((lowestcost).row != 0) { //checking bottom of parent
                float startheuristic = 0.0f;
                int pos = (lowestcost).row - 1;
                int ypos = lowestcost.col;
                checkingneighbours(pos, ypos, request, lowestcost, startheuristic);

            }
            //top
            if (lowestcost.row != width) { //checking top of parent
                float startheuristic = 0.0f;
                int ypos = lowestcost.col;
                int pos = (lowestcost).row + 1;
                checkingneighbours(pos, ypos, request, lowestcost, startheuristic);

            }
            //top left
            if (lowestcost.row != height && lowestcost.col != 0) { //checking for top left

                bool topwall = false;
                bool leftwall = false;

                if (lowestcost.row != width) { //checking if it within bounds
                int topcol = lowestcost.col;
                int toprow = (lowestcost).row + 1;
                topwall= terrain->is_wall(toprow, topcol);
                }
                if (lowestcost.col != 0) { //checking if its within bounds
                    int leftcol = (lowestcost).col - 1;
                    int leftrow = (lowestcost).row;
                    leftwall = terrain->is_wall(leftrow, leftcol);
                }

                if (topwall || leftwall) { //if there is a wall on either top of left
                    //do nothing
                }
                else { //if it is no walls go diagonal
                    float startheuristic = 0.0f;
                    int ypos = (lowestcost).col - 1;
                    int pos = (lowestcost).row + 1;
                    checkingdiagonals(pos, ypos, request, lowestcost, startheuristic);

                }
            }
            //left
            if (lowestcost.col != 0) { //checking for left of parent
                float startheuristic = 0.0f;
                int ypos = (lowestcost).col - 1;
                int pos = (lowestcost).row;
                checkingneighbours(pos, ypos, request, lowestcost, startheuristic);
            }
            //right
            if (lowestcost.col != height) { //checking for right of parent
                float startheuristic = 0.0f;
                int ypos = (lowestcost).col + 1;
                int pos = (lowestcost).row;
                checkingneighbours(pos, ypos, request, lowestcost, startheuristic);
            }
            //bottom right
            if (lowestcost.row != 0 && lowestcost.col != height) {//checking  bottom right
                bool rightwall = false;
                bool bottomwall = false;
                if (lowestcost.col != height) {
                    int rightcol = (lowestcost).col + 1;
                    int rightrow = (lowestcost).row;
                    rightwall = terrain->is_wall(rightrow, rightcol);
                }
                if ((lowestcost).row != 0) {
                    int bottomrow = (lowestcost).row - 1;
                    int bottomcol = lowestcost.col;
                    bottomwall = terrain->is_wall(bottomrow, bottomcol);
                }
                if (rightwall || bottomwall) { // if there walls on right and bottom, dont calculcate top bottomright
                    //do nothing bro
                }
                else {
                    float startheuristic = 0.0f;
                    int ypos = (lowestcost).col + 1;
                    int pos = (lowestcost).row - 1;
                    checkingdiagonals(pos, ypos, request, lowestcost, startheuristic);
                }
            }
            //bottom left
            if (lowestcost.row != 0 && lowestcost.col != 0) { //checking bottom left

                bool leftwall = false;
                bool bottomwall = false;

                if (lowestcost.col != 0) {
                    int leftcol = (lowestcost).col - 1;
                    int leftrow = (lowestcost).row;
                    leftwall = terrain->is_wall(leftrow, leftcol);
                }
                if ((lowestcost).row != 0) {
                    int bottomrow = (lowestcost).row - 1;
                    int bottomcol = lowestcost.col;
                    bottomwall = terrain->is_wall(bottomrow, bottomcol);
                }
                if (leftwall||bottomwall) {
                    //do nothing
                }
                else { //calulcate if both no walls
                    float startheuristic = 0.0f;
                    int ypos = (lowestcost).col - 1;
                    int pos = (lowestcost).row - 1;
                    checkingdiagonals(pos, ypos, request, lowestcost, startheuristic);
                }
            }
            //top right
            if (lowestcost.row != width && lowestcost.col != height) { //checking top right

                bool topwall = false;
                bool rightwall = false;

                if (lowestcost.row != width) {
                    int topcol = lowestcost.col;
                    int toprow = (lowestcost).row + 1;
                    topwall = terrain->is_wall(toprow, topcol);
                }
                if (lowestcost.col != height) {
                    int rightcol = (lowestcost).col + 1;
                    int rightrow = (lowestcost).row;
                    rightwall = terrain->is_wall(rightrow, rightcol);
                }

                if (topwall || rightwall) {
                    //do nothing
                }
                else {
                    float startheuristic = 0.0f;
                    int ypos = (lowestcost).col + 1;
                    int pos = (lowestcost).row + 1;
                    checkingdiagonals(pos, ypos, request, lowestcost, startheuristic);
                }
            }


  
        if (request.settings.singleStep == true) {
            return PathResult::PROCESSING;
        }


    }

   
    return PathResult::IMPOSSIBLE;
    
   



}
