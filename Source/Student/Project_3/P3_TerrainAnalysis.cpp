#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    
    float temp = 0;
    float toreturn = 1000.0f;
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        with cells outside the map bounds treated as walls, and return the smallest
        distance.  Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */

    for (int i =-1; i < 41; ++i) {
        for (int j = -1; j < 41; ++j) {
            //check if this position is a valid position, if true, proceed with calculation and storing into the array 
            if (terrain->is_valid_grid_position(i,j) == true) {
                //check if that cell is a wall
                if (terrain->is_wall(i,j)) {
                    float xdiff= abs(static_cast<float>(row - i));
                    float ydiff= abs(static_cast<float>(col - j));
                    temp= sqrt((xdiff * xdiff) + (ydiff * ydiff));

                    if (temp<toreturn) {
                        toreturn = temp;
                    }
                }
                else {
                    //do nothing
                }

            }
            else { //if the position is not valid, means it is out of bound walls
                float xdiff = abs(static_cast<float>(row - i));
                float ydiff = abs(static_cast<float>(col - j));
                temp = sqrt((xdiff * xdiff) + (ydiff * ydiff));

                if (temp < toreturn) {
                    toreturn = temp;
                }
            }
        }
    }

  


    //std::cout << toreturn << "\n";
    return toreturn;
  
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */

    // WRITE YOUR CODE HERE
    bool isnottopclear = false;
    bool isnotbotclear = false;
    bool isnotleftclear = false;
    bool isnotrightclear = false;

    Vec3 finddist1= terrain->get_world_position(0, 0);
    Vec3 finddist2= terrain->get_world_position(0, 1);
    //because .y gives 0
    float xdiff = abs(finddist1.x - finddist2.x);
    float ydiff = abs(finddist1.z - finddist2.z);
    float distofgrid = sqrt((xdiff* xdiff) + (ydiff*ydiff));

    int RowMin = std::min(row0, row1);
    int RowMax = std::max(row0, row1);

    int ColMin = std::min(col0, col1);
    int ColMax = std::max(col0, col1);

    
    Vec3 startpoint = terrain->get_world_position(row0, col0);
    Vec2 starto{};
    starto.x = startpoint.x;
    starto.y = startpoint.z;
    Vec3 goalpoint = terrain->get_world_position(row1, col1);
    Vec2 endo{};
    endo.x = goalpoint.x;
    endo.y = goalpoint.z;


    //needs to be max + 1 because max is the index of the row/column, not adding +1 will not let it go to those cells
    for (int i = RowMin; i < (RowMax+1);++i) {
        for (int j = ColMin; j < (ColMax+1); ++j) {
            //check the four sides of the wall
            if (terrain->is_wall(i, j) == true) {
                //get the vec2 of the wall, which will give the location of the middle of the wall
                Vec3 wallmidpoint = terrain->get_world_position(i, j);
                Vec2 getmid{};
                getmid.x = wallmidpoint.x;
                getmid.y = wallmidpoint.z;

                //all the points on the wall, 0.02 will be the slight padding
                Vec2 topleft{};
                topleft.x = getmid.x + (distofgrid/2) + 0.02f;
                topleft.y = getmid.y - (distofgrid / 2) - 0.02f;
                
                Vec2 topright{};
                topright.x = getmid.x + (distofgrid / 2)+ 0.02f;
                topright.y = getmid.y + (distofgrid / 2)+ 0.02f;

                Vec2 botleft{};
                botleft.x = getmid.x - (distofgrid / 2) - 0.02f;
                botleft.y = getmid.y - (distofgrid / 2) - 0.02f;

                Vec2 botright{};
                botright.x = getmid.x - (distofgrid / 2) - 0.02f;
                botright.y = getmid.y + (distofgrid / 2) + 0.02f;

                //check  top
                isnottopclear=line_intersect(starto, endo, topleft, topright);
                //check left
                isnotleftclear=line_intersect(starto, endo, topleft, botleft);
                //check right
                isnotrightclear=line_intersect(starto, endo, topright, botright);
                //check bottom
                isnotbotclear=line_intersect(starto, endo, botleft, botright);

                //if there is a line intersect, means is not a clear path
                if (isnotbotclear || isnottopclear || isnotleftclear || isnotrightclear) {
                    return false;
                }
            }
        }
    }


    return true; // REPLACE THIS
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.  Walls should not be marked.
    */
    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            if (terrain->is_wall(i, j) == false) {
                float dist = distance_to_closest_wall(i, j);
           // std::cout << dist << "\n";
                float toset = 1 / (dist * dist);
              //std::cout << toset << std::endl;
                layer.set_value(i, j, toset);
            }
        }
    }

    // WRITE YOUR CODE HERE
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */
   
    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();

    float count = 0.0f;

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            //if the cell is not a wall
            if (terrain->is_wall(i, j) == false) {
                //check this one grid with every other grid and itself(?)
                for (int checkingx = 0; checkingx < width; ++checkingx) {
                    for (int checkingy = 0; checkingy < height; ++checkingy) {
                        //if the path to that grid is clear 
                        if (is_clear_path(i, j, checkingx, checkingy)) {
                            count += 1.0f;
                        }

                    }
                }
                //now set the value of that grid and reset count
                float value = count / 160.0f;
                if (value > 1.0f) {
                    value = 1.0f;
                }
                layer.set_value(i, j, value);
                count = 0.0f;

            }


        }
    }

    //debuggging

   /* for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            if (is_clear_path(1, 0, i, j)) {
                count += 1.0f;
            }
        }
    }
    
    layer.set_value(1, 0, count);
    std::cout << layer.get_value(1, 0)<< std::endl;*/
    
    // WRITE YOUR CODE HERE
}
//visibility to agent
void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE

    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            //for every cell in given layer mark it with 1.0 and is not a wall
            if (terrain->is_wall(i, j) == false) {

                layer.set_value(i, j, 1.0f);
                //if its not visible 
                if (is_clear_path(row, col, i, j) == false) {
                    //checking neighbouring cells if they are value 1

                    //checking bottom
                    if (i != 0) {
                        if (layer.get_value(i - 1, j) == 1.0f) {
                            layer.set_value(i, j, 0.5f);
                            continue;
                        }
                    }

                    //checking left
                    if (j != 0) {
                        if (layer.get_value(i, j - 1) == 1.0f) {
                            layer.set_value(i, j, 0.5f);
                            continue;
                        }
                    }
                    //checking right
                    if (j != (height - 1)) {
                        if (layer.get_value(i, j + 1) == 1.0f) {
                            layer.set_value(i, j, 0.5f);
                            continue;
                        }
                    }
                    //checking top
                    if (i != (width - 1)) {
                        if (layer.get_value(i + 1, j) == 1.0f) {
                            layer.set_value(i, j, 0.5f);
                            continue;
                        }
                    }
                    //checking top left
                    if (i != (width - 1) && j != 0) {
                        // need to check if there is walls for diagonals
                        bool topwall = false;
                        bool leftwall = false;

                        if (i != (width - 1)) {
                            topwall = terrain->is_wall(i + 1, j);
                        }
                        if (j != 0) {
                            leftwall = terrain->is_wall(i, j - 1);
                        }

                        if (topwall || leftwall) {
                            //do nothing
                        }
                        else {
                            if (layer.get_value(i + 1, j - 1) == 1.0f) {
                                layer.set_value(i, j, 0.5f);
                                continue;
                            }
                        }

                    }
                    //checking bottom right
                    if (i != 0 && j != (height - 1)) {
                        bool rightwall = false;
                        bool bottomwall = false;

                        if (j != (height-1)) {
                            rightwall = terrain->is_wall(i,j+1);
                        }
                        if (i!=0) {
                            bottomwall = terrain->is_wall(i - 1, j);
                        }

                        if (rightwall || bottomwall) {
                            //do nothing
                        }
                        else {

                            if (layer.get_value(i - 1, j + 1) == 1.0f) {
                                layer.set_value(i, j, 0.5f);
                                continue;
                            }
                        }

                    }
                    //checking bottom left
                    if (i != 0 && j != 0) {

                        bool leftwall = false;
                        bool bottomwall = false;

                        if (j != 0) {
                            leftwall = terrain->is_wall(i, j - 1);
                        }
                        if (i != 0) {
                            bottomwall = terrain->is_wall(i - 1, j);
                        }

                        if (leftwall || bottomwall) {
                            //do nothing
                        }
                        else {

                            if (layer.get_value(i - 1, j - 1) == 1.0f) {
                                layer.set_value(i, j, 0.5f);
                                continue;
                            }
                        }

                    }
                    //checking top right
                    if (i != (width - 1) && j != (height - 1)) {

                        bool topwall = false;
                        bool rightwall = false;

                        if (i != (width-1) ) {
                            topwall = terrain->is_wall(i + 1, j);
                        }
                        if (j!=(height-1)) {
                            rightwall = terrain->is_wall(i, j + 1);
                        }

                        if (topwall || rightwall) {
                            //do nothing
                        }
                        else {

                            if (layer.get_value(i + 1, j + 1) == 1.0f) {
                                layer.set_value(i, j, 0.5f);
                                continue;
                            }
                        }

                    }

                    layer.set_value(i, j, 0.0f);
                }
            }
            else {
                layer.set_value(i, j, 0.0f);
            }
        }
    }

  /*  GridPos sendto3;
    sendto3.row = row;
    sendto3.col = col;
    Vec3 toprint = terrain->get_world_position(sendto3);
    std::cout << toprint.x << "   " << toprint.y << "    " << toprint.z << std::endl;*/
}

//this is search
void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();

    GridPos AgentGridPos = terrain->get_grid_position(agent->get_position());


    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            if (is_clear_path(AgentGridPos.row, AgentGridPos.col, i, j)) {
               

                Vec3 cell = terrain->get_world_position(i, j);
                Vec3 agentpos = agent->get_position();

                Vec3 AgentAndCell = agentpos - cell;
                AgentAndCell.Normalize();

                Vec3 viewvector= agent->get_forward_vector();
                viewvector.Normalize();

                //do dot product
                float result = (AgentAndCell.x * viewvector.x) + (AgentAndCell.z * viewvector.z);
                //0.09 because it will look exactly like the sample, 0.1 can see more
                if (result <= 0.09f) {
                    layer.set_value(i, j, 1.0f);
                }
                
                
            }
        }
    }
    
 
}

/************************************************** happens when propagation is on , always running ******************************************/
void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */
    
    //the tile with value 1 should not be changed, should be changed the last

   // width and height of the map
    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();


    float templayer[40][40]{};
    
    for (int row = 0; row < width;++row) {
        for (int col = 0; col < height;++col) {

            float biggestvalue = 0;

            if (terrain->is_wall(row, col) == false) {

                //checking bottom
                if (row != 0) {
                    float botvalue = layer.get_value(row - 1, col) * exp(-1 * decay);
                    
                    if (botvalue > biggestvalue) {                      
                        biggestvalue = botvalue;
                    }
                }
                //checking top
                if (row != (width - 1)) {
                    float topvalue = layer.get_value(row + 1, col) * exp(-1 * decay);
                    if (topvalue > biggestvalue) {
                        biggestvalue = topvalue;
                    }
                }

                //checking left
                if (col != 0) {
                    float leftvalue = layer.get_value(row, col - 1) * exp(-1 * decay);
                    if (leftvalue > biggestvalue) {
                        biggestvalue = leftvalue;
                    }
                }

                //checking right
                if (col != (height - 1)) {
                    float rightvalue = layer.get_value(row, col + 1) * exp(-1 * decay);
                    if (rightvalue > biggestvalue) {
                        biggestvalue = rightvalue;
                    }
                }


                //checking top left
                if (row != (width - 1) && col != 0) {
                    // need to check if there is walls for diagonals
                    bool topwall = false;
                    bool leftwall = false;

                    if (row != (width - 1)) {
                        topwall = terrain->is_wall(row + 1, col);
                    }
                    if (col != 0) {
                        leftwall = terrain->is_wall(row, col - 1);
                    }

                    if (topwall || leftwall) {
                        //do nothing
                    }
                    else {

                        float rightvalue = layer.get_value(row + 1, col - 1) * exp(-sqrt(2.0f) * decay);
                        if (rightvalue > biggestvalue) {
                            biggestvalue = rightvalue;
                        }

                    }
                }


                //checking bottom right
                if (row != 0 && col != (height - 1)) {
                    bool rightwall = false;
                    bool bottomwall = false;

                    if (col != (height - 1)) {
                        rightwall = terrain->is_wall(row, col + 1);
                    }
                    if (row != 0) {
                        bottomwall = terrain->is_wall(row - 1, col);
                    }

                    if (rightwall || bottomwall) {
                        //do nothing
                    }
                    else {

                        float rightvalue = layer.get_value(row - 1, col + 1) * exp(-sqrt(2.0f) * decay);
                        if (rightvalue > biggestvalue) {
                            biggestvalue = rightvalue;
                        }

                    }
                }

                //checking bottom left
                if (row != 0 && col != 0) {

                    bool leftwall = false;
                    bool bottomwall = false;

                    if (col != 0) {
                        leftwall = terrain->is_wall(row, col - 1);
                    }
                    if (row != 0) {
                        bottomwall = terrain->is_wall(row - 1, col);
                    }

                    if (leftwall || bottomwall) {
                        //do nothing
                    }
                    else {

                        float rightvalue = layer.get_value(row - 1, col - 1) * exp(-sqrt(2.0f) * decay);
                        if (rightvalue > biggestvalue) {
                            biggestvalue = rightvalue;
                        }
                       
                    }
                }

                //checking top right
                if (row != (width - 1) && col != (height - 1)) {

                    bool topwall = false;
                    bool rightwall = false;

                    if (row != (width - 1)) {
                        topwall = terrain->is_wall(row + 1, col);
                    }
                    if (col != (height - 1)) {
                        rightwall = terrain->is_wall(row, col + 1);
                    }

                    if (topwall || rightwall) {
                        //do nothing
                    }
                    else {

                        float rightvalue = layer.get_value(row + 1, col + 1) * exp(-sqrt(2.0f) * decay);
                        if (rightvalue > biggestvalue) {
                            biggestvalue = rightvalue;
                        }

                    }
                }

                float getvalue= lerp(layer.get_value(row,col), biggestvalue, growth);              
                templayer[row][col] = getvalue;
            }
        }
    }

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {           
                layer.set_value(i, j, templayer[i][j]);
        }
    }
    ////gets the influence layer of that tile
    //std::cout << layer.get_value(1, 0) << "\n";
    
    // WRITE YOUR CODE HERE
}

void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // WRITE YOUR CODE HERE
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */

    // WRITE YOUR CODE HERE


    const int width = terrain->get_map_width();
    const int height = terrain->get_map_height();

    float biggestval = 0;

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
                if (layer.get_value(i, j) > biggestval) {
                    biggestval = layer.get_value(i, j);
                }
        }
    }

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {        
                float valuetoset = (layer.get_value(i, j) / biggestval);
                if (valuetoset >= 0) {
                    layer.set_value(i, j, valuetoset);
                }
        }
    }

}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */

    // WRITE YOUR CODE HERE

   
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */

    // WRITE YOUR CODE HERE

    //std::cout << "this is it" << std::endl;
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

//only gets called once

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE

   /* std::cout << "THIS IS IT TOOOOOOOOOO" << std::endl;*/

    return false; // REPLACE THIS
}
