# Reflexive Layer Status Messages:
# -100 - waypoint not achieved, actuator state change occurred
#  100 - waypoint achieved, requesting next waypoint
#
# assumption is that new/next waypoint will be sent for all messages except 000
int16 refl_layer_status
# full waypoint list that was sent to RL from HL
nav_msgs/Path waypoints
# reference to waypoint ID in Path.header.seq (waypoint message /habitual/HL_to_HR/header/seq)
# -1 = no usable data, start over (ignore)
int32 seq
# reference to specific waypoint ID in poses.header.seq (waypoint in waypoint message /habitual/HL_to_RL/waypoints/poses[waypoint_seq])
# -1 = no usable data, start over (ignore)
int32 waypoint_seq # this can be from a list of waypoints; currently only one waypoint per message
