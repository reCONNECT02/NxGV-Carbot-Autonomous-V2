uint8 id                # 1..13
string name
uint8 max_marks
string state            # PENDING | ACTIVE | PASS | FAIL
string level_estimate   # EXCELLENT | COMPLETED | PARTIAL | FAIL | ""
string evidence         # why (events / contact / intervention)
float32 t_start_s
float32 t_end_s
