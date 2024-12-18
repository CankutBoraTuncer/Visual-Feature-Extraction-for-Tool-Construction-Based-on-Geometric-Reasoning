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
    model_names = os.listdir(model_path)

    for model_name in model_names:
        model_name = model_name.split(".")[0]
        print("Model name:", model_name)
        base_arg = "X: [0.0, 0.0, 0.2, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../src/models/tools/simple/parts/"
        arg = base_arg + model_name + ".stl>,"

        C = ry.Config()
        C.addFile("../src/config/base.g")
        C.addFrame(model_name, "world", args=arg)  

        r = RAI(C, cam_list, view=False)
        pts = r.get_point_cloud(model_name, filter = filter, img_view = False)

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud("../src/point_clouds_ref/" + model_name + ".ply", point_cloud)
