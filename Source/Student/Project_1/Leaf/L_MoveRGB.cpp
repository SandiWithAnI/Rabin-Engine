#include <pch.h>
#include "L_MoveRGB.h"

L_MoveRGB::L_MoveRGB() : timer(0.0f)
{}

void L_MoveRGB::on_enter()
{
    thecolor.x = 0.0f;
    thecolor.y = 0.0f;
    thecolor.z = 0.0f;
    timer = 1.0f;
    targetPoint = RNG::world_position();
    BehaviorNode::on_leaf_enter();
}

void L_MoveRGB::on_update(float dt)
{
    //RNG color, values to ensure it usually stays dark and visible
    thecolor.x = RNG::range(0.0f, 80.0f);
    thecolor.y = RNG::range(0.0f, 10.0f);
    thecolor.z = RNG::range(0.0f, 20.0f);


    //for the agent to change color
    timer -= dt;
    //the agent to move to random point
    const auto result = agent->move_toward_point(targetPoint, dt);
    //change the color
    if (timer <= 0.0f)
    {
        agent->set_color(thecolor);
        //reset the timer
        timer = 1.0f;
    }
    //if the agent reaches the point
    if (result == true) {
        on_success();
    }

    display_leaf_text();
}