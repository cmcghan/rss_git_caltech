nav_msgs/Path waypoints
float64 risk
# handling: 0 = drop prev. waypoints/commands and run this now/replace with this
#           1 = add this to the end of stack
#           2 = pause current and add this to the start of the stack
uint32 handling
