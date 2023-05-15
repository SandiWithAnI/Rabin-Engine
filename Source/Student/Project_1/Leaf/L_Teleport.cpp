#include <pch.h>
#include "L_Teleport.h"
#include "Agent/BehaviorAgent.h"

void L_Teleport::on_enter()
{
    // set animation, speed, etc

    targetPoint = RNG::world_position();
    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_Teleport::on_update(float dt)
{

    //set the position of the agent
    agent->set_position(targetPoint);
    //the angle which the agent is facing
    float rotation = RNG::range(-180.0f, 180.0f);
    agent->set_yaw(rotation);

    on_success();
    

    display_leaf_text();
}