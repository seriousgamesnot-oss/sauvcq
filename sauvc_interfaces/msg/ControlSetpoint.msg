std_msgs/Header header

uint8 MODE_IDLE=0
uint8 MODE_GOTO_POSITION=1
uint8 MODE_HOLD_POSITION=2
uint8 control_mode

geometry_msgs/Point desired_position   # EKF frame, metres
float64 desired_depth                  # metres, positive down
float64 desired_yaw                    # radians
