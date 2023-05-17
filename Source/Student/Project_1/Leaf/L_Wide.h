#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_Wide : public BaseNode<L_Wide>
{
public:
    L_Wide();
protected:
    float timer;//timer for the size change
    float colortimer;//timer for the RGB change
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
    Vec3 thecolor;
};