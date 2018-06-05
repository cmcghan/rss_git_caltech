# Habitual Layer Status Messages:
# 000 - no message(, continue current processes)
# 100 - waypoint achieved
# 200 - waypoint not achieved, unknown error
# 210 - waypoint not achieved, some_constraint requirement will fail
# 211 - waypoint not achieved, some_constraint requirement has failed
# 220 - waypoint not achieved, time requirement will fail
# 221 - waypoint not achieved, time requirement has failed
# 231 - waypoint not achieved, actuator degradation detected
# 232 - waypoint not achieved, actuator failure detected
#
# assumption is that new/next waypoint will be sent for all messages except 000
int16 hab_layer_status
# full waypoint list that was sent to HL from DL
nav_msgs/Path waypoints
# reference to waypoint ID in Path.header.seq (waypoint message /deliberative/DL_to_HL/header/seq)
# -1 = no usable data, start over (ignore)
int32 seq
# reference to specific waypoint ID in poses.header.seq (waypoint in waypoint message /deliberative/DL_to_HL/waypoints/poses[waypoint_seq])
# -1 = no usable data, start over (ignore)
int32 waypoint_seq # this can be from a list of waypoints; currently only one waypoint per message
# Additional amount of some(_constraint) level requested for completion of waypoint
# 0 = no additional request for some_constraint_reading_level
# -1 = indicates unknown amount of some_constraint_reading_level will be necessary, like a HL poke for a DL-level replan request
# positive number = approximate lower bound on expected some(_constraint) level after future/additional consumption
float64 some_request
# Additional amount of time requested for completion of waypoint
# 0 = no additional request for time
# -1 = indicates unknown amount of time will be necessary, like a HL poke for a DL-level replan request
# positive number = approximate lower bound on expected time after future/additional time of completion
float64 time_request
# Additional rethink requested due to actuator state change
# 0 = not prompted by actuator state change
# -1 = indicates unknown issue
# (int) = indicates which type of actuator degrade/failure prompted this, like a HL poke for a DL-level replan request
float64 actuator_request
