Format of the 3D graph files:

Every line in the file specifies either one vertex or one edge

The vertices are specified as follws:
VERTEX3 id x y z phi theta psi  

The edges are specified as follows:
EDGE3 observing_vertex_id observed_vertex_id x y z roll pitch yaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66 

(the information matrix is specified via its upper triangular block that means 21 values).
