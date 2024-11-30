import sys
sys.path.append('../src')
import os
import pyvista as pv
from SEG import SEG

point_cloud_path = "../src/point_clouds_ref"
model_names = os.listdir(point_cloud_path)

for model_name in model_names:
    print("Model name:", model_name)
    point_cloud = pv.read(point_cloud_path + "/" + model_name)

    tps = SEG(verbose = 1)
    #tps.segment_point_cloud(point_cloud)
    tps.segment_point_cloud_v2(point_cloud_path + "/" + model_name)
