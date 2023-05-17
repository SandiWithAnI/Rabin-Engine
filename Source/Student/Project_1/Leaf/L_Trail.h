#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_Trail : public BaseNode<L_Trail>
{
protected:
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
};