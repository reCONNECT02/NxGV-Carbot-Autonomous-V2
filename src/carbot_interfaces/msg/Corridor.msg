# Block 09 -> block 10 and GUI main tab (lane lines). /carbot/plan/corridor
std_msgs/Header header  # frame_id odom
nav_msgs/Path guide     # route-conditioned guide, shifted by the observed offset
geometry_msgs/Point[] centres   # measured lane centres (z = measured width)
geometry_msgs/Point[] left_edge
geometry_msgs/Point[] right_edge
float32 offset_m
uint32 observed
string mode             # CAMERA CORRIDOR | CAMERA ROAD + ROUTE BRANCH | REMEMBERED CORRIDOR
bool lane_locked        # main tab paints lane lines blue when true
bool branch_hold
string branch_label
string branch_reason
