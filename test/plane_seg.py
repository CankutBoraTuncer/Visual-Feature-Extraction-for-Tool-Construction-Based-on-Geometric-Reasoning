import sys
sys.path.append('../src')
import os

from RAI import RAI
import robotic as ry
import pyvista as pv
from SEG import SEG

if __name__ == "__main__":
    cam_list = ["cam_front", "cam_back", "cam_left", "cam_right", "cam_up"]
    filter = 1
    model_path = "../src/models/tools/simple/parts"
    model_names = os.listdir(model_path)

    C = ry.Config()
    C.addFile("../src/config/plane.g")
    C.addFile("../src/config/base.g")

    dx = -0.1
    dy = -0.1
    c = 0
    for model_name in model_names:
        model_name = model_name.split(".")[0]
        print("Model name:", model_name)

        if(c % 4 == 0):
            dx = -0.1
            dy+= 0.1
        else:
            dx += 0.1
        c+=1

        base_arg = "X: [" + str(dx) + ", " + str(dy) + ", 0.2, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../src/models/tools/simple/parts/"
        arg = base_arg + model_name + ".stl>,"
        C.addFrame(model_name, "world", args=arg)  

    r = RAI(C, cam_list, view=False)
    pcl, _, _ = r.get_raw_point_cloud(img_view=False, filter = filter)
    tps = SEG(verbose = 1)
    pcl_filtered = tps.RANSAC_plane(pcl)
    tps.segment_objects(pv.PolyData(pcl_filtered))
