#pragma once
#include "BehaviorNode.h"

class D_RepeatSuccess : public BaseNode<D_RepeatSuccess>
{
public:
    D_RepeatSuccess();

protected:
    unsigned counter;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};