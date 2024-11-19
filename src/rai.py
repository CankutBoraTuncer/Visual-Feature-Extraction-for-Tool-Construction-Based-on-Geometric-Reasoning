import robotic as ry
import numpy as np
import matplotlib.pyplot as plt

class rai():

    def __init__(self, C, cam_list, view=False):
        self.C = C
        self.cam_list = cam_list
        self.view = view
        
        if(view):
            self.C.view(True)
    

    def get_point_cloud(self, object_name):
        pts_w = np.array([])

        for cam_frame in self.cam_list:
            camera_view = ry.CameraView(self.C)
            cam = self.C.getFrame(cam_frame)
            camera_view.setCamera(cam)
            img, depth = camera_view.computeImageAndDepth(self.C)
            img = np.asarray(img)
            depth = np.asarray(depth)
            seg = camera_view.computeSegmentationImage()
            filter_id = int(self.C.getFrame(object_name).info()["ID"])
            filter_color = self.id2color(filter_id)

            masked_img = img.copy()
            masked_depth = depth.copy()

            for i in range(masked_img.shape[0]):
                for j in range(masked_img.shape[1]):
                    if (seg[i][j] == filter_color).all() == False:
                        masked_img[i][j] = [0, 0, 0]
                        masked_depth[i][j] = 0

            
            pts = ry.depthImage2PointCloud(masked_depth, camera_view.getFxycxy())
            pts = self.cam_to_world(pts.reshape(-1, 3), cam)
            pts_w = np.append(pts_w, pts)
            
            if(self.view):
                fig = plt.figure()
                fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
                fig.add_subplot(1,3,1)
                plt.imshow(img)
                fig.add_subplot(1,3,2)
                plt.imshow(masked_img)
                fig.add_subplot(1,3,3)
                plt.imshow(seg)
                plt.show()

        if(self.view):
            C_view = ry.Config()
            C_view.addFrame("world")
            C_view.getFrame("world").setPointCloud(pts_w, [0,0,0])
            C_view.view(True)

        return pts_w

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


    @staticmethod
    def id2color(id):
        rgb = [0, 0, 0]
        rgb[0] = (((id >> 6) & 0x3F) | ((id & 1) << 7) | ((id & 8) << 3))
        rgb[1] = (((id >> 12) & 0x3F) | ((id & 2) << 6) | ((id & 16) << 2)) 
        rgb[2] = (((id >> 18) & 0x3F) | ((id & 4) << 5) | ((id & 32) << 1)) 
        return np.asarray(rgb)
    
    @staticmethod
    def addMarker(C, pos, parent, name, size, is_relative, quat = [1, 0, 0, 0]):
        marker = C.addFrame(name, parent= parent) \
                .setShape(ry.ST.marker, size = [size]) \
                .setQuaternion(quat)
        if is_relative:
            marker.setRelativePosition(pos)
        else:
            marker.setPosition(pos)

    @staticmethod
    def addPoint(C, pos, parent, name, size, is_relative, quat = [1, 0, 0, 0]):
        marker = C.addFrame(name, parent= parent) \
                .setShape(ry.ST.marker, size = [size]) \
                .setQuaternion(quat)
        if is_relative:
            marker.setRelativePosition(pos)
        else:
            marker.setPosition(pos)