#include <pch.h>
#include "L_Sizes.h"
#include "Agent/BehaviorAgent.h"

L_Sizes::L_Sizes() : timer(0.0f)
{}


void L_Sizes::on_enter()
{
    // set animation, speed, etc

    targetPoint = RNG::world_position();
    timer = 0.5f;

    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_Sizes::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);
    //for agent to change size
    timer -= dt;
    float size = RNG::range(1.0f, 10.0f);

    if (timer <= 0.0f)
    {
        agent->set_scaling(size);
        //reset the timer
        timer = 0.5f;
    }

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}