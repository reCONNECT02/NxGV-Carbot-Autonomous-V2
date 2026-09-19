# Block 13 (and tunnel bridge / recovery / parking trackers) -> block 15.
# Every requester publishes on its own topic /carbot/request/<source>.
# The command owner only listens to the source that mission logic marks active,
# and rejects any request older than command_owner.request_expiry_s.
std_msgs/Header header  # stamp = time the request was computed
string source           # ROAD | TUNNEL | PARKING | RECOVERY | HOLD
float32 speed_mps       # signed: + forward, - reverse
float32 steer_rad       # front-wheel angle, REP-103: + = turn LEFT
bool arrived            # tracker reached the end of its path
string reason
