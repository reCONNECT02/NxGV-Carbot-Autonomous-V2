uint8 index             # 1..12
string id               # key in calibration_steps.yaml
string title
string status           # PENDING | RUNNING | PASS | FAIL | KEPT_PREVIOUS | SKIPPED_OPTIONAL
bool required
bool can_advance
string result_summary
string result_file
string previous_session # session the kept value came from
