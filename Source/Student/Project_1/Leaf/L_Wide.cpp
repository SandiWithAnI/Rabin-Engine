#include <pch.h>
#include "L_Wide.h"
#include "Agent/BehaviorAgent.h"

L_Wide::L_Wide() : timer(0.0f), colortimer(0.0f)
{}


void L_Wide::on_enter()
{
    // set animation, speed, etc
    agent->set_movement_speed(15.0f);

    thecolor.x = 0.0f;
    thecolor.y = 0.0f;
    thecolor.z = 0.0f;
    targetPoint = RNG::world_position();
    colortimer = 0.5f;
    timer = 1.0f;

    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_Wide::on_update(float dt)
{

    //RNG color, values to ensure it usually stays dark and visible
    thecolor.x = RNG::range(0.0f, 20.0f);
    thecolor.y = RNG::range(0.0f, 10.0f);
    thecolor.z = RNG::range(0.0f, 80.0f);

    Vec3 thesize{};
    thesize.x = RNG::range(1.0f, 40.0f);
    thesize.y = 3.0f;
    thesize.z = 3.0f;
    const auto result = agent->move_toward_point(targetPoint, dt);
    //for agent to change size
    timer -= dt;
    colortimer -= dt;
 

    if (timer <= 0.0f)
    {
        agent->set_scaling(thesize);
        //reset the timer
        timer = 1.0f;
    }
    if (colortimer<=0.0f) {
        agent->set_color(thecolor);
        //reset the timer
        colortimer = 0.5f;
    }

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}