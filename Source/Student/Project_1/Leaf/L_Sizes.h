#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_Sizes : public BaseNode<L_Sizes>
{
public:
    L_Sizes();
protected:
    float timer;
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
};