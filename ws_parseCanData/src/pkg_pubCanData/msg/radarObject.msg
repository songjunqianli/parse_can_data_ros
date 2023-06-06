# std_msgs/Header header
# 7 "RESERVED" 6 "RESERVED" 5 "RESERVED" 4 "CROSSING" 3 "MOVING" 2 "STOPPED" 1 "STATIONARY" 0 "UNKNOWN"
uint8 MotionPattern
float32 YPos_Stdev
float32 XVelRel_Stdev
float32 XPos_Stdev
float32 XAccRel
# 1 "OBJECT_VALID" 0 "OBJECT_INVALID"
uint8 ValidFlag
# 1 "OBJECT_EXISTED_IN_PREVIOUS_CYCLE" 0 "NEW_OBJECT_INTHE_CYCLE"
uint8 UpdateFlag
float32 ObstacleProb
# 255 "INVALID"
uint16 ID
float32 ExstProb
int16 YVelRel
int16 YPos
int16 XVelRel
int16 XPos
# 3 "PEDESTRAIN" 2 "TWO_WHEELER" 1 "FOUR_WHEELER" 0 "UNKNOWN"
uint8 ObjType
# 1 "OBJECT_EXTRAPOLATED_THIS_CYCLE" 0 "OBJECT_MEASURED_INTHIS_CYCLE"
uint8 MeasFlag