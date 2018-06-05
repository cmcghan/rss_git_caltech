# waypoint in waypoint message (/simulator/waypoint_list)
nav_msgs/Path waypoint
# handling: 0 = drop prev. waypoints/commands and run this now/replace with this
#           1 = add this to the end of stack
#           2 = pause current and add this to the start of the stack
uint32 handling
# Flag true or false (0=false, 1=true)
#bool some_flag
#uint32 some_flag
# Flag_true true or false ('no'=false, 'yes'=true)
string Flag_true
