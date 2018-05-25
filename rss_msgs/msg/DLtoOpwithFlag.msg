# Deliberative Layer Status Messages:
# 000 - no message(, continue current processes)
# 100 - waypoint sent
# 200 - waypoint not sent, unknown error / failure, shut down
# 900 - requesting more waypoints from operator
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
# Flag true or false (0=false, 1=true)
#bool some_flag
#uint32 some_flag
# Flag_true true or false ('no'=false, 'yes'=true)
string Flag_true
