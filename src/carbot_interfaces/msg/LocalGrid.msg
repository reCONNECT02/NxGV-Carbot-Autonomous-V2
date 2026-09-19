# Block 03 (camera road grid, vehicle frame) and block 04 (memory, odom frame).
# Cell kinds follow V4 perception.js: 0 unseen, 1 road, 2 paint, 3 other.
uint8 UNSEEN=0
uint8 ROAD=1
uint8 PAINT=2
uint8 OTHER=3
std_msgs/Header header  # frame_id base_link (block 03) or odom (block 04)
float32 resolution_m
float32 origin_x_m      # x of cell (row 0) lower edge, V4 x0
float32 origin_y_m      # y of cell (col 0) lower edge, V4 y0
uint32 rows             # along +x
uint32 cols             # along +y
uint8[] kind            # rows*cols, row-major
uint8[] grown           # 1 = connected road region (4-neighbour growth from seed)
uint8[] source_camera   # 0 none, 1 front, 2 left_rear, 3 right_rear (block 03 only)
float32[] age_s         # block 04 only: age of each cell
float32 coverage        # fraction of cells seen by any camera
uint32 connected        # number of grown road cells
