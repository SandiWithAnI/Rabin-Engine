#pragma once
#include "BehaviorNode.h"
#include "Misc/NiceTypes.h"

class L_BoarderRun : public BaseNode<L_BoarderRun>
{
public:
    //L_BoarderRun();
protected:
    //float timer;
    virtual void on_enter() override;
    virtual void on_update(float dt) override;

private:
    Vec3 targetPoint;
    int number = 1;
};