import sys
sys.path.append('../src')

from rai import rai
import robotic as ry
import numpy as np
import matplotlib.pyplot as plt

object_dict = {
    "hammer_handle": "X: [0.0, 0.0, 1, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: 1, shape: mesh, mesh: <../LIRA_models/stl/hammer_handle.stl>, visual: True",
    "hammer_head_large": "X: [0, 0, 0.9, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: 1, shape: mesh, mesh: <../LIRA_models/stl/hammer_head_large.stl>, visual: True"
}

if __name__ == "__main__":
    cam_list = ["cam_front", "cam_back", "cam_left", "cam_right", "cam_up", "cam_down"]
    object_name = "hammer_handle"
    C = ry.Config()
    
    C.addFile("../src/config/base.g")
    C.addFrame(object_name, "world", args=object_dict[object_name])  
    
    r = rai(C, cam_list, view=True)
    pts = r.get_point_cloud(object_name, 0.5)
    print(pts.shape)

