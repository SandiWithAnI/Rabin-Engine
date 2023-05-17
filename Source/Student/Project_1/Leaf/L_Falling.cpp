#include <pch.h>
#include "L_Falling.h"
#include "Agent/BehaviorAgent.h"

void L_Falling::on_enter()
{
    // set animation, speed, etc

   

    Vec3 inthesky;
    inthesky.x = RNG::world_position().x;
    inthesky.y = 100.0f;
    inthesky.z = RNG::world_position().z;
    agent->set_position(inthesky);

    targetPoint.x = inthesky.x;
    targetPoint.y = -400.0f;
    targetPoint.z = inthesky.z;
    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_Falling::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    //when the agent has successfully fallen.
    if (result == true)
    {
        //generate a random position on the map
        Vec3 finalpos;
        finalpos.x = RNG::world_position().x;
        finalpos.y = RNG::world_position().y;
        finalpos.z = RNG::world_position().z;

        on_success();
        agent->set_position(finalpos);
    }

    display_leaf_text();
}
