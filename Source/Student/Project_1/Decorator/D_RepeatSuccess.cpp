#include <pch.h>
#include "D_RepeatSuccess.h"

D_RepeatSuccess::D_RepeatSuccess() : counter(0)
{}

void D_RepeatSuccess::on_enter()
{
    counter = 0;
    BehaviorNode::on_enter();
}

void D_RepeatSuccess::on_update(float dt)
{
    
    BehaviorNode* child = children.front();

    child->tick(dt);

    if (child->succeeded() == true)
    {
        on_success();
    }
    else if (child->failed() == true)
    {
        //repeat until succeed, to set child to go into on enter()
        child->set_status(NodeStatus::READY);
    }
}
