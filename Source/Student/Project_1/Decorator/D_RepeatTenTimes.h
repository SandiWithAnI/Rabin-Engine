#pragma once
#include "BehaviorNode.h"

class D_RepeatTenTimes : public BaseNode<D_RepeatTenTimes>
{
public:
    D_RepeatTenTimes();

protected:
    unsigned counter;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};