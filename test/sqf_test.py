import sys
sys.path.append('../src')
import open3d as o3d
from SQF import SQF

file_path = "/home/bora/Bilkent_MS/CS554_Computer_Vision/CS554_Computer_Vision_Project/src/point_clouds_ref/axe_handle_s.ply"
pcd = o3d.io.read_point_cloud(file_path)
sqf = SQF(pcd)
sqf.fit()

