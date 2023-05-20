#pragma once
#include "BehaviorNode.h"

class D_AlwaysFail : public BaseNode<D_AlwaysFail>
{
protected:
    virtual void on_update(float dt) override;
};