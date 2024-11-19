import sys
sys.path.append('../src')

from rai import rai
import robotic as ry
import numpy as np
import matplotlib.pyplot as plt

object_dict = {
    "hammer_handle": "X: [0.0, 0.0, 1, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: -1, shape: mesh, mesh: <../../LIRA_models/stl/hammer_handle.stl>, visual: True",
    "hammer_head_large": "X: [0.0, 0.0, 1, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: -1, shape: mesh, mesh: <../../LIRA_models/stl/hammer_head_large.stl>, visual: True"
}
if __name__ == "__main__":
    cam_list = ["cam_frame_0", "cam_frame_1", "cam_frame_2", "cam_frame_3", "cam_frame_4", "cam_frame_5"]
    r = rai("../src/config/base.g", cam_list, view=True)
    r.C.addFrame("hammer_head_large", "world", object_dict["hammer_head_large"])
    r.get_point_cloud("hammer_head_large")
    