#include <pch.h>
#include "L_FloorSpin.h"
#include "Agent/BehaviorAgent.h"


void L_FloorSpin::on_enter()
{

    timer = 5.0f;
    agent->set_roll(PI/2.0f);
    agent->set_pitch(-PI/2.0f);

    BehaviorNode::on_leaf_enter();
}

void L_FloorSpin::on_update(float dt)
{
    timer -= dt;

    float rotation = RNG::range(-PI, PI);
    agent->set_yaw(rotation);
      
    if (timer <= 0.0f) {
        agent->set_roll(0.0f);
        agent->set_pitch(0.0f);
        on_success();
        timer = 5.0f;
    }
        


    display_leaf_text();
}
