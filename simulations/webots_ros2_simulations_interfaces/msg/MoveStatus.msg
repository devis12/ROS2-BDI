# Move message used to communicate the current status of a moving operation

# @start_name           -> name of the starting position 
# @current_name         -> name of the current position 
# @target_name          -> name of the target position 
# @target_pos           -> target position point
# @current_pos          -> current position point
# @dist                 -> distance on a straight line
# @euclidean_dist       -> current euclidean distance between 2 aforementioned points
# @progress             -> progress in [0-1] 


string                  start_name
string                  current_name
string                  target_name
geometry_msgs/Point     target_pos
geometry_msgs/Point     current_pos
float32                 dist
float32                 euclidean_dist
float32                 progress