# A configuration of the generalized Dubins vehicle in 3D.
#
# The orientation is given by a right-handed orthonormal frame:
#   tangent        (T) - direction of travel
#   tangent_normal (t) - "wing" direction, T x t = u
#   surface_normal (u) - normal of the osculating surface
# The roll angle of the vehicle is encoded by the rotation of (t, u) about T.

geometry_msgs/Point position
geometry_msgs/Vector3 tangent
geometry_msgs/Vector3 tangent_normal
geometry_msgs/Vector3 surface_normal
