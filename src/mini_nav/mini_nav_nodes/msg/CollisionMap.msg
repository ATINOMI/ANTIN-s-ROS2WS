# 覆盖/静态格与连续动态端点原子发布，grid.header 为观测时间和共同坐标系。
nav_msgs/OccupancyGrid grid
geometry_msgs/Point[] points
float64 observation_uncertainty
float64 clearance_radius
bool valid
# 实时端点可留在 odom；空值表示与 grid 同帧，保留原同帧保守预算。
string points_frame_id
