import sys
sys.path.append('../src')

from rai import rai
import robotic as ry
import numpy as np
import matplotlib.pyplot as plt

object_dict = {
    "hammer_handle": "X: [0.0, 0.0, 1, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: -1, shape: mesh, mesh: <../LIRA_models/stl/hammer_handle.stl>, visual: True",
    "hammer_head_large": "X: [0.0, 0.0, 1, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: -1, shape: mesh, mesh: <../LIRA_models/stl/hammer_head_large.stl>, visual: True"
}
if __name__ == "__main__":
    cam_list = ["cam_front", "cam_back", "cam_left", "cam_right", "cam_up", "cam_down"]
    C = ry.Config()
    C.addFile("../src/config/base.g")
    C.addFrame("hammer_head_large", "world", object_dict["hammer_head_large"])
    r = rai(C, cam_list, view=True)
    r.get_point_cloud("hammer_head_large")
    