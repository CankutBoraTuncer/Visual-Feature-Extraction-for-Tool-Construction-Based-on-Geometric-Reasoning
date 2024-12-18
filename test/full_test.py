import sys
sys.path.append('../src')
import os

from RAI import RAI
import robotic as ry
import open3d as o3d







if __name__ == "__main__":
    reference_tool = "knife" # Select a reference tool
    reference_tool_path = "../src/models/tools/simple/parts/matched/" + reference_tool

    # Get point cloud
    C = ry.Config()
    C.addFile("../src/config/base.g")

    base_arg = "X: [0.0, 0.0, 0.2, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../src/models/tools/simple/parts/matched/"
    arg = base_arg + reference_tool + ".stl>,"
    C.addFrame(reference_tool, "world", args=arg)  

    r = RAI(C, cam_list, view=False)
    pts = r.get_point_cloud(model_name, filter = filter, img_view = False)