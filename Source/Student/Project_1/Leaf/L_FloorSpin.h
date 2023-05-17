#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_FloorSpin : public BaseNode<L_FloorSpin>
{
protected:
    float timer;
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:

};