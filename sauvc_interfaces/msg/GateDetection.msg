std_msgs/Header header          # frame_id = optical frame of the camera that produced this
string camera_id                # "front" or "bottom"
float32 confidence

bool has_position               # true if perception gives a metric 3D pose
geometry_msgs/Point position    # metres, in header.frame_id (valid if has_position)
geometry_msgs/Quaternion orientation

bool has_bbox                   # true if perception gives a 2D box
float32 x_min                   # pixels
float32 y_min
float32 x_max
float32 y_max
