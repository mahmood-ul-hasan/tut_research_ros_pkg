from graphslam.load import load_g2o_se3
import matplotlib.pyplot as plt


g = load_g2o_se3("/home/aisl/catkin_ws/src/latest_crane/map_optimization/plot/before_opti_graph.g2o") 


g.plot(vertex_markersize=1)

g.calc_chi2()

g.optimize()

g.plot(vertex_markersize=1)

