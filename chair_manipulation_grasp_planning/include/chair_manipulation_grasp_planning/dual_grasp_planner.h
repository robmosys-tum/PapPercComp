#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H

#include "grasp_planner.h"

namespace chair_manipulation
{

class DualGraspPlanner
{
public:
    DualGraspPlanner(GraspPlanner & planner1, GraspPlanner & planner2)
        : planner1(planner1),
          planner2(planner2)
    {}

    void lift_chair();

private:
    GraspPlanner & planner1;
    GraspPlanner & planner2;
};

}

#endif //CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H
