string class_name       # traffic_light_red | traffic_light_green | speed_bump_sign | boom_gate_open | boom_gate_closed | ...
float32 confidence
string camera_role      # front | left_rear | right_rear
int32 x                 # bbox in the source image (pixels)
int32 y
int32 width
int32 height
float32 distance_m      # estimated (-1 unknown)
geometry_msgs/Point position    # estimated in base_link (for the main-tab 3D render)
