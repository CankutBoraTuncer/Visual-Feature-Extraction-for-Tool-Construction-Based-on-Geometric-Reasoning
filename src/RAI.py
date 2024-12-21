import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
import os
import open3d as o3d

class RAI():

    def __init__(self, verbose=0):
        self.verbose = verbose

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def create_tool(self, scene, match_1, match_2):
        _, candidate_1 = match_1
        _, candidate_2 = match_2

        attachment_1 = "at_" + "_".join(candidate_1.split("_")[1:])
        attachment_2 = "at_" + "_".join(candidate_2.split("_")[1:])

        self.addMarker(scene, [0, 0, 0], "l_l_gripper", "home", 0.0001, True)

        S = ry.Skeleton()
        S.enableAccumulatedCollisions(True)

        S.addEntry([0.1, -1], ry.SY.touch, ["l_l_gripper", candidate_1])
        S.addEntry([0.3, -1], ry.SY.stable, ["l_l_gripper", candidate_1])
        S.addEntry([0.8, -1], ry.SY.poseEq, [attachment_2, attachment_1])
        S.addEntry([1, -1], ry.SY.stable, [candidate_2, candidate_1])

        S.addEntry([1.1, -1], ry.SY.touch, ["l_l_gripper", candidate_2])
        S.addEntry([1.3, -1], ry.SY.stable, ["l_l_gripper", candidate_2])
        S.addEntry([1.8, -1], ry.SY.poseEq, ["home", "l_l_gripper"])

        komo = S.getKomo_path(scene, stepsPerPhase=30, accScale=1e0, lenScale=1e-2, homingScale=1e-1, collScale=1e3)
        ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
        print(ret)
        scene.view(True)
        komo.view_play(pause=True, delay=0.5)
        return ret.feasible
    
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def generate_ref_point_cloud(self, model_path, cam_list, filter = 1, target_path = "", object_name=""):
        model_names = os.listdir(model_path)
        obj_c = 0
        ptc_arr = {}
        for model_name in model_names:
            model_name = model_name.split(".")[0]

            if object_name != "" and object_name not in model_name:
                continue

            if object_name != "" and object_name in model_name:
                obj_c+=1
            
            if obj_c > 2: 
                return ptc_arr

            print("Model name:", model_name)
            base_arg = "X: [0.0, 0.0, 0.3, 0, 0, 1, 0], color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <"+model_path+"/"
            arg = base_arg + model_name + ".stl>,"
            
            C = ry.Config()
            C.addFile("../src/config/base.g")
            C.addFrame(model_name, "world", args=arg)  

            pts = self.get_point_cloud(C, cam_list, model_name, filter = filter)

            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(pts)

            if target_path != "":
                o3d.io.write_point_cloud(target_path + model_name + ".ply", point_cloud)

            ptc_arr[model_name] = point_cloud

        return ptc_arr
    
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def get_point_cloud(self, C, cam_list, object_name, filter=1):
        pts_w = np.array([])

        for cam_frame in cam_list:
            camera_view = ry.CameraView(C)
            cam = C.getFrame(cam_frame)
            camera_view.setCamera(cam)
            img, depth = camera_view.computeImageAndDepth(C)
            img = np.asarray(img)
            depth = np.asarray(depth)
            seg = camera_view.computeSegmentationImage()

            filter_id = int(C.getFrame(object_name).info()["ID"])
            filter_color = self.id2color(filter_id)

            masked_img = img.copy()
            masked_depth = depth.copy()
            
            for i in range(masked_img.shape[0]):
                for j in range(masked_img.shape[1]):
                    seg[i][j] = [x + 1 if x % 2 != 0 else x for x in seg[i][j]]
                    if (seg[i][j] == filter_color).all() == False:
                        masked_img[i][j] = [0, 0, 0]
                        masked_depth[i][j] = 0
            
            pts = ry.depthImage2PointCloud(masked_depth, camera_view.getFxycxy())
            pts = self.cam_to_world(pts.reshape(-1, 3), cam)
            pts_w = np.append(pts_w, pts)
            
            if(self.verbose > 1):
                fig = plt.figure()
                fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
                fig.add_subplot(1,3,1)
                plt.imshow(img)
                fig.add_subplot(1,3,2)
                plt.imshow(masked_img)
                fig.add_subplot(1,3,3)
                plt.imshow(seg)
                plt.show()
        
        camera_positions = np.array([
            C.getFrame("cam_front").getPosition(),
            C.getFrame("cam_back").getPosition(),
            C.getFrame("cam_left").getPosition(),
            C.getFrame("cam_right").getPosition(),
            C.getFrame("cam_up").getPosition(),
            C.getFrame("cam_down").getPosition()
        ])

        pts_w = pts_w.reshape(-1, 3)
        z_mask = pts_w[:, 2] <= 0.5
        pts_w = pts_w[z_mask]
        mask = ~np.any(np.all(pts_w[:, None, :] == camera_positions[None, :, :], axis=2), axis=1)
        pts_w_f = pts_w[mask].flatten()

        if(self.verbose>1):
            C_view = ry.Config()
            C_view.addFrame("world")
            C_view.getFrame("world").setPointCloud(pts_w_f, [0,0,0])
            C_view.view(True)

        pts_w = np.asarray(pts_w_f).reshape(-1, 3)
        row_indices = np.random.choice(pts_w.shape[0], size=int(pts_w.shape[0]*filter), replace=False)
        pts_w = pts_w[row_indices]
        return pts_w

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def get_raw_point_cloud(self, C, cam_list, filter=1):
        pts_w = np.array([])

        for cam_frame in cam_list:
            camera_view = ry.CameraView(C)
            cam = C.getFrame(cam_frame)
            camera_view.setCamera(cam)
            img, depth = camera_view.computeImageAndDepth(C)
            img = np.asarray(img)
            depth = np.asarray(depth)

            pts = ry.depthImage2PointCloud(depth, camera_view.getFxycxy())
            pts = self.cam_to_world(pts.reshape(-1, 3), cam)
            pts_w = np.append(pts_w, pts)
            
            if(self.verbose > 1):
                fig = plt.figure()
                fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
                fig.add_subplot(1,2,1)
                plt.imshow(img)
                fig.add_subplot(1,2,2)
                plt.imshow(depth)
                plt.show()
        
        camera_positions = np.array([
            C.getFrame("cam_front").getPosition(),
            C.getFrame("cam_back").getPosition(),
            C.getFrame("cam_left").getPosition(),
            C.getFrame("cam_right").getPosition(),
            C.getFrame("cam_up").getPosition(),
            C.getFrame("cam_down").getPosition()
        ])

        pts_w = pts_w.reshape(-1, 3)
        z_mask = pts_w[:, 2] <= 0.5
        pts_w = pts_w[z_mask]
        mask = ~np.any(np.all(pts_w[:, None, :] == camera_positions[None, :, :], axis=2), axis=1)
        pts_w_f = pts_w[mask].flatten()

        if(self.verbose>1):
            C_view = ry.Config()
            C_view.addFrame("world")
            C_view.getFrame("world").setPointCloud(pts_w_f, [0,0,0])
            C_view.view(True)

        pts_w = np.asarray(pts_w_f).reshape(-1, 3)
        row_indices = np.random.choice(pts_w.shape[0], size=int(pts_w.shape[0]*filter), replace=False)
        pts_w = pts_w[row_indices]
        return pts_w, img, depth

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def read_point_cloud_from_folder(pointcloud_folder):
        pointcloud_dict = {}
        for filename in os.listdir(pointcloud_folder):
            if filename.endswith(".ply"):
                file_path = os.path.join(pointcloud_folder, filename)
                pcd = o3d.io.read_point_cloud(file_path)
                base_name = os.path.splitext(filename)[0]
                pointcloud_dict[base_name] = pcd
        
        return pointcloud_dict   

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def point2obj(scene, pts):
        objects = scene.getFrameNames()
        for obj in objects:
            if "obj" in obj:
                for p in pts:
                    if np.linalg.norm(scene.getFrame(obj).getPosition() - p) < 0.1:
                        return obj
        return ""

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def view_point_cloud(pts, is_flat = False):
        if not is_flat:
            pts = pts.flatten()
        C = ry.Config()
        C.addFrame("world")
        C.getFrame("world").setPointCloud(pts, [0,0,0])
        C.view(True)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def cam_to_world(pts, cam_frame):
        t = cam_frame.getPosition() 
        R = cam_frame.getRotationMatrix()
        points_camera_frame = pts
        points_camera_frame_homogeneous = np.hstack((points_camera_frame, np.ones((points_camera_frame.shape[0], 1))))
        transformation_matrix = np.vstack((np.hstack((R, t.reshape(-1, 1))), np.array([0, 0, 0, 1])))
        points_world_frame_homogeneous = np.dot(transformation_matrix, points_camera_frame_homogeneous.T).T
        points_world_frame = points_world_frame_homogeneous[:, :3]
        return points_world_frame

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def id2color(id):
        rgb = [0, 0, 0]
        rgb[0] = ((id >> 6) & 0x3F) | ((id & 1) << 7) | ((id & 8) << 3)
        rgb[1] = ((id >> 12) & 0x3F) | ((id & 2) << 6) | ((id & 16) << 2)
        rgb[2] = ((id >> 18) & 0x3F) | ((id & 4) << 5) | ((id & 32) << 1)
        return np.asarray(rgb)
    
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def color2id(rgb):
        id = 0
        id |= ((rgb[0] & 0x80) >> 7) | ((rgb[1] & 0x80) >> 6) | ((rgb[2] & 0x80) >> 5) 
        id |= ((rgb[0] & 0x40) >> 3) | ((rgb[1] & 0x40) >> 2) | ((rgb[2] & 0x40) >> 1) 
        id |= ((rgb[0] & 0x3F) << 6) | ((rgb[1] & 0x3F) << 12) | ((rgb[2] & 0x3F) << 18)  
        return id

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    @staticmethod
    def addMarker(C, pos, parent, name, size, is_relative, quat = [1, 0, 0, 0]):
        marker = C.addFrame(name, parent= parent) \
                .setShape(ry.ST.marker, size = [size]) \
                .setQuaternion(quat)
        if is_relative:
            marker.setRelativePosition(pos)
        else:
            marker.setPosition(pos)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#
    
    @staticmethod
    def addPoint(C, pos, parent, name, size, is_relative, quat = [1, 0, 0, 0]):
        marker = C.addFrame(name, parent= parent) \
                .setShape(ry.ST.marker, size = [size]) \
                .setQuaternion(quat)
        if is_relative:
            marker.setRelativePosition(pos)
        else:
            marker.setPosition(pos)