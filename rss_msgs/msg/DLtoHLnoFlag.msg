# Deliberative Layer Status Messages:
# 000 - no message(, continue current processes)
# 100 - waypoint sent
# 200 - waypoint not sent, unknown error / failure, shut down
#
# assumption is that new/next waypoint will be sent for all received HabLayer messages except 000
int16 delib_layer_status
# data from old /simulator/waypoint_list topic, now included
nav_msgs/Path waypoints
# reference to waypoint ID in Path.header.seq (waypoint message /simulator/waypoint_list)
uint32 seq
# reference to specific waypoint ID in poses.header.seq (waypoint in waypoint message /simulator/waypoint_list)
#uint32 waypoint_seq # this can be a list of waypoint_seq and some_constraint; currently only one waypoint per message
# bound on some(_constraint) level at end of waypoint following for current waypoint
float64 some_constraint
# handling: 0 = drop prev. waypoints/commands and run this now/replace with this
#           1 = add this to the end of stack
#           2 = pause current and add this to the start of the stack
uint32 handling
