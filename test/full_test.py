import sys
sys.path.append('../src')
import numpy as np
import robotic as ry

from RAI import RAI
from SEG import SEG
from SQF import SQF
from Score import Score

if __name__ == "__main__":
    cam_list = ["cam_front", "cam_back", "cam_left", "cam_right", "cam_up", "cam_down"]
    filter = 1
    model_path = "../src/models/tools/simple/parts"
    target_path = "../src/point_clouds_ref/"
    scene = ry.Config()
    scene.addFile("../src/config/tools_simple1_s.g")
    scene.view(True)

    reference_tool = "rake" # Select a reference tool
    candidate_objects = ["axe_handle_s", "hammer_head_s", "rake_head_s", "screwdriver_head_s", "hammer_handle_s"]

    # STEP 1: Generate the point cloud for the reference tool
    rai = RAI(verbose=0)
    segments_ref = rai.generate_ref_point_cloud(model_path, cam_list, filter = filter, object_name=reference_tool)

    # STEP 2: Generate the point cloud for the candidate objects
    cam_list = ["cam_front", "cam_back", "cam_right", "cam_up"]
    ptc_cand, _, _ = rai.get_raw_point_cloud(scene, cam_list, filter = filter)
   
    # STEP 3: Remove the plane from the scene
    seg = SEG(verbose=0)
    pcl_filtered = seg.RANSAC_plane(ptc_cand)
    segments_cand = seg.segment_objects(pcl_filtered)

    # STEP 2.5 (Optional): Read the pointclouds from the folder (the dataset)
    #segments_cand = RAI.read_point_cloud_from_folder("../src/point_clouds_aux/")

    # STEP 4: Fit SuperQuadric function to the objects
    param_ref  = {}
    for key, pcd in segments_ref.items():
        sqf = SQF(pcd, verbose=0)
        param = sqf.fit()
        if len(param) < 12:
            param = np.pad(param, (0, 12 - len(param)), mode='constant', constant_values=0)
        param_ref[key] = param

    param_cand = {}
    for key, pcd in segments_cand.items():
        sqf = SQF(pcd, verbose=0)
        param = sqf.fit()
        if len(param) < 12:
            param = np.pad(param, (0, 12 - len(param)), mode='constant', constant_values=0)
        param_cand[key] = param
        
    # STEP 5: Find the best matching candidates objects to the reference tool set
    score = Score(param_ref, param_cand, candidate_objects, verbose=1)
    match, _, _ = score.find_best_fit()
    for r, c in match:
        print("Candidate")
        RAI.view_point_cloud(np.asarray(segments_cand[c].points))
        print("Reference")
        RAI.view_point_cloud(np.asarray(segments_ref[r].points))

    # STEP 6: Create the tool
    