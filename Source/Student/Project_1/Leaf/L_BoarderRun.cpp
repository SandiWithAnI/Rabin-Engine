#include <pch.h>
#include "L_BoarderRun.h"
#include "Agent/BehaviorAgent.h"

//L_BoarderRun::L_BoarderRun() : timer(0.0f)
//{}


void L_BoarderRun::on_enter()
{
    // set animation, speed, etc
    if (number == 1) {
    targetPoint.x = 97.0f;
    targetPoint.y = 0.0f;
    targetPoint.z = 2.0f;
    number += 1;
    }
    else if (number==2) {
        targetPoint.x = 97.0f;
        targetPoint.y = 0.0f;
        targetPoint.z = 97.0f;
        number += 1;
    }
    else if (number==3) {
        targetPoint.x = 2.0f;
        targetPoint.y = 0.0f;
        targetPoint.z = 97.0f;
        number += 1;
    }
    else if (number==4) {
        targetPoint.x = 2.0f;
        targetPoint.y = 0.0f;
        targetPoint.z = 2.0f;
        number = 1;
    }
    //agent->look_at_point(targetPoint);
    agent->set_movement_speed(50.0f);


    BehaviorNode::on_leaf_enter();
}

void L_BoarderRun::on_update(float dt)
{

    
        const bool isdone = agent->move_toward_point(targetPoint, dt);
        if (isdone == true) {
            //++number;
            on_success();
        }

    display_leaf_text();
}
