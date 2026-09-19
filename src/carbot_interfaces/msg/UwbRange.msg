string anchor_id        # hex string as sent by the tag, e.g. "1786"
float32 range_raw_m     # links[].R
float32 range_corrected_m   # raw - offset, flattened by anchor/tag z difference
uint32 age_ms           # links[].age_ms
uint32 sample_seq       # links[].sample_seq
bool fresh              # false if sample_seq already used (repeat) or too old
builtin_interfaces/Time measured_stamp  # arrival_time - age_ms
bool gated_out          # set by block 06 when the innovation gate rejected it
