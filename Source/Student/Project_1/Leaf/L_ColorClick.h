#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_ColorClick : public BaseNode<L_ColorClick>
{
protected:
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 thecolor;
};