import sys
sys.path.append('../src')
import os

from RAI import RAI
import robotic as ry
import open3d as o3d


if __name__ == "__main__":
    cam_list = ["cam_front", "cam_back", "cam_left", "cam_right", "cam_up", "cam_down"]
    filter = 1
    model_path = "../src/models/tools/simple/parts"
    target_path = "../src/point_clouds_ref/"
    rai = RAI(verbose=0)
    rai.generate_point_cloud(model_path, cam_list, filter = filter, target_path = target_path)