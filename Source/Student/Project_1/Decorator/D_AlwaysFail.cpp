#include <pch.h>
#include "D_AlwaysFail.h"

void D_AlwaysFail::on_update(float dt)
{
    BehaviorNode* child = children.front();

    child->tick(dt);

    if (child->succeeded() == true)
    {
        on_failure();
    }
    else if (child->failed() == true)
    {
        on_failure();
    }

}


