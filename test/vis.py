import robotic as ry
import sys
sys.path.append('../src')

C = ry.Config()
C.addFile("../src/config/tools_simple_all.g")
C.watchFile("../src/config/tools_simple_all.g")