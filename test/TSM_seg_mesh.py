import sys
sys.path.append('../src')
import os
import pyvista as pv
from SEG import SEG

model_path = "../src/models"
model_names = os.listdir(model_path)
for model_name in model_names:
    print("Processing Model:", model_name)
    mesh = pv.read(os.path.join(model_path, model_name))

    tps = SEG(verbose = 1)
    tps.segment_mesh(mesh)