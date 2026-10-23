std_msgs/Header header          # frame_id = bottom camera optical frame
string camera_id                # "bottom"
bool valid                      # false if perception cannot estimate floor distance
float32 clearance_m             # metres from vehicle to pool floor
float32 confidence
