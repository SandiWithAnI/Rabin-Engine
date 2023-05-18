#include <pch.h>
#include "L_ColorClick.h"
#include "Agent/BehaviorAgent.h"

void L_ColorClick::on_enter()
{
    // set animation, speed, etc

    thecolor.x = 0.0f;
    thecolor.y = 0.0f;
    thecolor.z = 0.0f;
   

    //agent->look_at_point(targetPoint);

    BehaviorNode::on_leaf_enter();
}

void L_ColorClick::on_update(float dt)
{
    thecolor.x = RNG::range(0.0f, 80.0f);
    thecolor.y = RNG::range(0.0f, 20.0f);
    thecolor.z = RNG::range(0.0f, 5.0f);

    //check if there is a click
    const auto leftMouseState = InputHandler::get_current_state(MouseButtons::LEFT);
    //check if the state of the click is a pressed
    if (leftMouseState == InputHandler::InputState::PRESSED)
    {
        agent->set_color(thecolor);
        on_success();
    }
    else {
        on_failure();
    }

    display_leaf_text();
}