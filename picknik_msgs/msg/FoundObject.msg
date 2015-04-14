# Name of the object found in the scene
string object_name

# The bounding box/shape,this is in the pose frame
shape_msgs/Mesh bounding_mesh

# The poses of all the objects expected in the scene,
geometry_msgs/Pose object_pose

# Value between 0 and 1 for each expected object's confidence of its pose
float32 expected_object_confidence