# Human detection message
# Contains 3D bounding box information for detected humans in world frame

std_msgs/Header header

# Detection ID for tracking
uint32 id

# 3D bounding box center in world frame
geometry_msgs/Point center

# 3D bounding box dimensions (width, height, depth)
geometry_msgs/Vector3 size

# Orientation of the bounding box
geometry_msgs/Quaternion orientation

# Velocity of the detected human
geometry_msgs/Vector3 velocity

# Confidence score [0.0, 1.0]
float32 confidence

# Number of points in the cluster
uint32 num_points
