#pragma once
#include "BehaviorNode.h"

class L_MoveRGB : public BaseNode<L_MoveRGB>
{
public:
    L_MoveRGB();

protected:
    float timer;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
    Vec3 thecolor;
};