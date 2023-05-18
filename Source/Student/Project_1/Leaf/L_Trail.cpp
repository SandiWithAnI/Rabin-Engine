#include <pch.h>
#include "L_Trail.h"
#include "Agent/BehaviorAgent.h"

void L_Trail::on_enter()
{
    // set animation, speed, etc
    targetPoint = RNG::world_position();
   
    agent->set_movement_speed(30.0f);
   // agent->set_roll(0.0f);
    //agent->set_pitch(0.0f);
    //agent->set_yaw(0.0f);
    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_Trail::on_update(float dt)
{

    const auto result = agent->move_toward_point(targetPoint, dt);
    //converting the player position to grid position
    GridPos start = terrain->get_grid_position(agent->get_position());
    //set the tile of the floor to random color
    terrain->set_color(start, RNG::color(1.0f));

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
