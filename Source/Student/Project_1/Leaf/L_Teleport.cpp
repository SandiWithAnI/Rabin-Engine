#include <pch.h>
#include "L_Teleport.h"
#include "Agent/BehaviorAgent.h"

void L_Teleport::on_enter()
{
    // set animation, speed, etc

    targetPoint = RNG::world_position();
    agent->set_movement_speed(100);
    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_Teleport::on_update(float dt)
{
    //const auto result = agent->move_toward_point(targetPoint, dt);
    agent->set_position(targetPoint);

        on_success();
    

    display_leaf_text();
}