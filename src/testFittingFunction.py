import numpy as np
import open3d as o3d
import math
import SQF
from scipy.optimize import least_squares
import numpy as np
import open3d as o3d
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize, Bounds
import copy
import matplotlib.pyplot
import numpy as np
import matplotlib.pyplot as plt

# Define the path to your .ply file
file_path = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/edited_pcl.ply' 

pcd = o3d.io.read_point_cloud(file_path)
points = np.asarray(pcd.points)

pcd_MATLAB = o3d.io.read_point_cloud('/Users/hsimsir/Documents/MATLAB/fitted_superquadric.ply')
points_MATLAB = np.asarray(pcd_MATLAB.points)

# Superquadric type
SQ_type = 3

x_init = np.array([
    0.06161033189955391, 0.11922311476483836, 0.5097239528361709, 0.02,
    0.1, 1.0, 0.015842171469100705, 1.5717786304102472, -0.008135844327065224,
    0.5101328209876588, -0.0007222597651816321, -0.0005506903443385312
])

lower_bound = np.array([
    np.float64(0.04928826551964313), np.float64(0.09537849181187069), np.float64(0.40777916226893673), np.float64(0.006),
    0.1, 0.1, -3.141592653589793, -3.141592653589793, -3.141592653589793,
    np.float64(-0.019447905672341687), np.float64(-0.11983178097185217), np.float64(-0.06154851318302501)
])
upper_bounds = np.array([
    np.float64(0.07393239827946468), np.float64(0.14306773771780604), np.float64(0.611668743403405), np.float64(0.034),
    2.0, 2.0, 3.141592653589793, 3.141592653589793, 3.141592653589793,
    np.float64(1.0), np.float64(0.11861444855782456), np.float64(0.06167215061608281)
])

# x_init = np.array([
#     0.3320644709399563, 0.999999127668082, 0.9999994717659741, 0.0,
#     0.1, 1.0, 9.999999814587539e-07, 1.5707963267949034, 9.324212254300996e-07,
#     9.99999998283202e-07, 9.9999999714538e-07, 9.99999999220413e-07
# ])

# # Minimum bounds for the parameters
# lower_bound = np.array([
#     np.float64(0.26565157675196505), np.float64(0.7999993021344656), np.float64(0.7999995774127794), np.float64(-0.01),
#     0.1, 0.1, -math.pi, -math.pi, -math.pi,
#     np.float64(-0.9999992589663207), np.float64(-1.0), np.float64(-0.3320645194626838)
# ])

# # Maximum bounds for the parameters
# upper_bounds = np.array([
#     np.float64(0.3984773651279475), np.float64(1.1999989532016984), np.float64(1.199999366119169), np.float64(0.01),
#     2.0, 2.0, math.pi, math.pi, math.pi,
#     np.float64(0.9999996845656277), np.float64(0.9999982553361639), np.float64(0.3320644224172287)
# ])

if not np.all(lower_bound <= upper_bounds):
    raise ValueError("Each element of min_bound must be less than or equal to max_bound.")

# Define the residual function
def residuals_fn(params, points, SQ_type):
    return SQF.fitting_fn(params, points, SQ_type)

# Perform optimization using least_squares
result = least_squares(
    fun=residuals_fn,
    x0=x_init,
    bounds=(lower_bound, upper_bounds),
    args=(points, SQ_type),
    method='trf',  # Trust Region Reflective algorithm, suitable for large problems
    max_nfev=5000,
    ftol=1e-8,
    xtol=1e-8,
    verbose=1
)

def objective_fn(params):
    cost = SQF.fitting_fn(params, points, SQ_type)
    return np.sum(cost)

bound = Bounds(lower_bound, upper_bounds)
# Perform optimization using only bounds
# result = minimize(
#     objective_fn,
#     x0=x_init,
#     bounds=bound,
#     method='SLSQP',
#     options={'disp': True, 'maxiter': 5000, 'ftol': 1e-8}
# )

# Check optimization success
# if not result.success:
#     raise ValueError(f"Optimization failed: {result.message}")

# Extract the optimized parameters
optimum_quadrics = result.x
print(f"optimum_quadrics: {optimum_quadrics}")
SQ = SQF.SQ2PCL(optimum_quadrics, SQ_type)
SQF.visualizePointClouds([SQ, points_MATLAB, points], ["Our SQFitting", "Baseline SQFitting", "Original Point Cloud"], title=f"Point cloud for SQ type: {SQF.SQTYPES[SQ_type]}")

# OUR IMPLEMENTATION PARAMS SQ_TYPE=0
# [ 5.89156007e-02  1.03261849e-01  4.89765051e-01 -9.99999980e-03
#   1.00000218e-01  4.70974881e-01 -1.88936556e-03  1.57194960e+00
#  -8.46939175e-03  4.94795904e-01  2.36367679e-03 -1.38731068e-03]

# MATLAB PARAMS: 0.0589    0.1033    0.4901         0    0.1015    0.4718   -0.0019    1.5719   -0.0086    0.4948    0.0024   -0.0014


