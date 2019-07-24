# PlanningTraj.msg
# Created on: Dec 7, 2016
#     Author: linpenghong

Header header
PlanningPoint[] trajectory
bool emergency
int32 decision
bool direction
bool new_trajectory
int32 type

int32 TO_CURRENT_LANE = 0
int32 TO_LEFT_LANE = 1
int32 TO_RIGHT_LANE = 2
int32 SOFT_STOP = 3
int32 EMERGENCY_STOP = 4
