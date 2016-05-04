# TimeOptimalPlanner

Computes the optimal cost of servicing a given set of waypoints considering any time penalties associated with missing each waypoint, and the time required for the robot to service each waypoint.  The penalty and service times are meant to represent the tradeoff between the robot's effort and any human effort needed to service each waypoint if the robot chooses not to visit it.  The robot is taken to be holonomic, moving with a user-specified velocity, V.  The robot is required to wait a user-specified time, D (dwell time) at each waypoint. 

The waypoints and costs are given in a input file consisting of multiple sets of waypoints, with each run preceded by an integer, N, indicating the number of waypoints in that run.  The N lines that follow are triples (X, Y, P) indicating the Cartesian coordinate, (X, Y), of the waypoint, and a time penalty, P, incurred if the waypoint is missed.

The function creates an output file of the same name as the input file (with an .out extension) containing a list of N costs (time durations) required to complete each run.
