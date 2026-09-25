# A feasible 3D Dubins path produced by the surface-based construction.

# Identifier of the winning construction, e.g. "cyc_inner", "plane_left_right",
# "sphere_outer".
string path_type

# Arc length of the path in metres.
float64 length

# Samples along the path, ordered from the initial to the final configuration.
asr_sdm_control_msgs/PathSample[] samples
