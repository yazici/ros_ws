# Message contains PhaseSpace (OWL) marker data

# ID of the marker
uint32 id

# ID of tracker the marker is assigned to
uint32 tracker_id

# Name assigned to the marker
string name

# String of options assigned to the marker (e.g. position on rigid object)
string options

# Status flags
uint32 flags

# 3D position of the marker
geometry_msgs/Point position

# Condition of the marker data (lower positive -> good, negative -> invalid)
float32 condition
