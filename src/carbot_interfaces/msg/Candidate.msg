uint32 id
float32 offset_m
geometry_msgs/Point[] points    # z = direction (+1 forward, -1 reverse)
float32 cost
bool valid
bool relaxed            # evaluated with the paint allowance
bool selected
float32 min_clear_m
float32 command_steer_rad
string stage            # parking: "15 cm docking straight" etc.
string reject           # rejection reason (GUI hover text)
