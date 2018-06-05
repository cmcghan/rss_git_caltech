# Actuator State Status Messages:
# 0 - nominal (fine / functional)
# 1 - degradation sensed
# 2 - further degradation sensed
# 3 - failure sensed
#
# integer made up of flags, overall actuator type-1 status
uint32 overall_status_actuator_type_1s
# if any type-1 actuator has failed, this will note it: 0000 means all working
# order of actuator flags for generic four-wheeled rover: FL,FR,BL,BR
# order of actuator flags for generic manipulator arm: basejoint0,joint1,joint2,gripperjoint

# integer made up of flags, overall actuator type-2 status
uint32 overall_status_actuator_type_2s
# if any type-2 actuator has failed, this will note it: 0000 means all working
# order of actuator flags for generic four-wheeled rover: FL,FR,BL,BR
# order of actuator flags for generic manipulator arm: basejoint0,joint1,joint2,gripperjoint

# actuator types can be all sorts of things!
# e.g., most rover actuators are motors for drive/trans vs. turn/rot for each wheel
# e.g., most manipulator arms are motors for revolute vs. prismatic motion for each joint

# specific messages for each actuator as list:
uint16[8] actuator
# order of actuators:
# (FL) front-left:  type-1, type-2
# (FR) front-right: type-1, type-2
# (BL)  back-left:  type-1, type-2
# (BR)  back-right: type-1, type-2
