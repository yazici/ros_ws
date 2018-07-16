# Message contains PhaseSpace (OWL) rigid data

# ID of the rigid
uint32 id

# Name assigned to the rigid (i.e. name of the rigid tracker)
string name

# Status flags
uint32 flags

# 3D pose of the rigid
geometry_msgs/Pose pose

# Condition of the rigid data (higher positive -> good, negative -> invalid)
float32 condition

# Is this a camera?
bool is_camera
