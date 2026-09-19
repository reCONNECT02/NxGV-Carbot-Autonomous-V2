# Blocks 05/06 -> GUI Localization tab, safety (block 14). /carbot/localization/status
std_msgs/Header header
float32 local_sigma_m   # V4 Estimator.sigma (lateral confidence)
float32 global_sigma_m  # V4 Estimator.globalSigma
float32 global_offset_x_m
float32 global_offset_y_m
float32 uwb_residual_m
float32 uwb_gain
bool uwb_last_accepted
float32 uwb_age_s
uint32 visual_matches
float32 visual_rank
bool visual_enabled
float32 distance_travelled_m
