std_msgs/Header header
string robot_ns
int32 grid_x_num
int32 grid_y_num
int32 grid_z_num
float64 cell_size
float64 cell_height
float64 origin_x
float64 origin_y
float64 origin_z
int8[] cell_status   # 0=UNSEEN 1=EXPLORING 2=COVERED 3=COVERED_BY_OTHERS 4=NOGO
int32[] exploring_cell_indices
geometry_msgs/Point[] exploring_cell_positions
