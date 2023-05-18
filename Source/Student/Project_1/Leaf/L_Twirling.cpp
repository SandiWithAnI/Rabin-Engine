#include <pch.h>
#include "L_Twirling.h"
#include "Agent/BehaviorAgent.h"

void L_Twirling::on_enter()
{
    // set animation, speed, etc

    targetPoint = RNG::world_position();

    //agent->look_at_point(targetPoint);
    agent->set_movement_speed(30.0f);

    BehaviorNode::on_leaf_enter();
}

void L_Twirling::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);
    float rotation = RNG::range(-PI, PI);
    agent->set_yaw(rotation);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
