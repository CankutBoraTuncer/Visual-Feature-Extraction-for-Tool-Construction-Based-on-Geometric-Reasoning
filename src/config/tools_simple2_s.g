world: {},

table (world){
    shape:ssBox, Q: "t(0 0. 0)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact: 1
}

ob_plane(table){}

obj_prism_long(ob_plane): {X: [-0.1 0.0 0.06 0.7 0 0.7 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/prism_long.stl>}
at_prism_long(obj_prism_long) : {X: [-0.1 -0.1 0.06, 1 0 0 0], shape: marker, size: [0.001]}

obj_disk(ob_plane): {X: [0.0 0.1 0.07 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/disk.stl>}
at_disk(obj_disk) : {X: [0.0 0.1 0.08, 1 0 0 0], shape: marker, size: [0.001]}

obj_sphere_half_hollow(ob_plane): {X: [0.1 -0.05 0.07 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/sphere_half_hollow.stl>}
at_sphere_half_hollow(obj_sphere_half_hollow) : {X: [0.1 -0.03 0.065, 1 0 0 0], shape: marker, size: [0.001]}

obj_prism_short(ob_plane): {X: [0.2 0.1 0.07 0.7 0 -0.7 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/prism_short.stl>}
at_prism_short(obj_prism_short) : {X: [0.18 0.1 0.07, 0.7 0 0 0.7], shape: marker, size: [0.001]}

cam_front(world): { rel: [0, 1, 1,  0.00000000,   0.00000000,  -0.92387953,   0.38268344 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_back(world): { rel: [0, -1, 1,    0.38268343,  -0.92387953,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_right(world): { rel: [-1, 0.1, 1,   0.27059805,  -0.65328148,   0.65328148,  -0.27059805 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  },
cam_left(world): { rel: [1, 0.1, 1,   0.27059805,  -0.65328148,  -0.65328148,   0.27059805 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  },
cam_up(world): { rel: [0, 0, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  },
cam_down(world): { rel: [0, 0, -1, 1,  0, 0, 0], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  }
cam_front_right(world): { rel: [-0.65, 0.65, 1,   0.17337590,  -0.35808816,   0.85149587,  -0.34156764], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_front_left(world): { rel: [0.75, 0.75, 1,   0.17987373,  -0.37692368,  -0.84218296,   0.34103055 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_back_right(world): { rel: [-0.65, -0.65, 1,   0.34935254,  -0.85318381,   0.35729011,  -0.14957931 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_back_left(world): { rel: [0.65, -0.65, 1,   0.34606604,  -0.85506437,  -0.35627669,   0.14889642 ], shape: camera, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },


l_panda_base: { multibody: True },
l_panda_link0(l_panda_base): { rel: [-0.5, 0, 0.05, 1, 0, 0, 0] },
l_panda_link0_0(l_panda_link0): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link0.ply>, visual: True },
l_panda_joint1_origin(l_panda_link0): { rel: [0, 0, 0.333, 1, 0, 0, 0] },
l_panda_joint1(l_panda_joint1_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link1(l_panda_joint1): {  },
l_panda_link1_0(l_panda_link1): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link1.ply>, visual: True },
l_panda_joint2_origin(l_panda_link1): { rel: [0, 0, 0, 0.707107, -0.707107, 0, 0] },
l_panda_joint2(l_panda_joint2_origin): { rel: [0, 0, 0, 0.877583, 0, 0, -0.479426], joint: hingeZ, limits: [-1.7628, 1.7628, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link2(l_panda_joint2): {  },
l_panda_link2_0(l_panda_link2): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link2.ply>, visual: True },
l_panda_joint3_origin(l_panda_link2): { rel: [0, -0.316, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint3(l_panda_joint3_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link3(l_panda_joint3): {  },
l_panda_link3_0(l_panda_link3): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link3.ply>, visual: True },
l_panda_joint4_origin(l_panda_link3): { rel: [0.0825, 0, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint4(l_panda_joint4_origin): { rel: [0, 0, 0, 0.540302, 0, 0, -0.841471], joint: hingeZ, limits: [-3.0718, -0.0698, 2.175, -1, 87, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] },
l_panda_link4(l_panda_joint4): {  },
l_panda_link4_0(l_panda_link4): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link4.ply>, visual: True },
l_panda_joint5_origin(l_panda_link4): { rel: [-0.0825, 0.384, 0, 0.707107, -0.707107, 0, 0] },
l_panda_joint5(l_panda_joint5_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
l_panda_link5(l_panda_joint5): {  },
l_panda_link5_0(l_panda_link5): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link5.ply>, visual: True },
l_panda_joint6_origin(l_panda_link5): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint6(l_panda_joint6_origin): { rel: [0, 0, 0, 0.540302, 0, 0, 0.841471], joint: hingeZ, limits: [0.5, 3, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
l_panda_link6(l_panda_joint6): {  },
l_panda_link6_0(l_panda_link6): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link6.ply>, visual: True },
l_panda_joint7_origin(l_panda_link6): { rel: [0.088, 0, 0, 0.707107, 0.707107, 0, 0] },
l_panda_joint7(l_panda_joint7_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] },
l_panda_link7(l_panda_joint7): {  },
l_panda_link7_0(l_panda_link7): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/link7.ply>, visual: True },
l_panda_joint8_origin(l_panda_link7): { rel: [0, 0, 0.107, 1, 0, 0, 0] },
l_panda_joint8(l_panda_joint8_origin): {  },
l_panda_link8(l_panda_joint8): {  },
l_panda_hand_joint_origin(l_panda_link8): { rel: [0, 0, 0, 0.92388, 0, 0, -0.382683] },
l_panda_hand_joint(l_panda_hand_joint_origin): {  },
l_panda_hand(l_panda_hand_joint): {  },
l_panda_hand_0(l_panda_hand): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/hand.ply>, visual: True },
l_panda_finger_joint1_origin(l_panda_hand): { rel: [0, 0, 0.0584, 1, 0, 0, 0] },
l_panda_finger_joint2_origin(l_panda_hand): { rel: [0, 0, 0.0584, 1, 0, 0, 0] },
l_panda_finger_joint1(l_panda_finger_joint1_origin): { rel: [0, 0.045, 0, 1, 0, 0, 0], joint: transY, limits: [0, 0.04, 0.2, -1, 20, 0.2, -1, 20], ctrl_limits: [0.2, -1, 20] },
l_panda_finger_joint2(l_panda_finger_joint2_origin): { rel: [-0, -0.045, -0, -1, 0, 0, 0], joint: transY, joint_scale: -1, limits: [0, 0.04, 0.2, -1, 20, 0.2, -1, 20], mimic: "l_panda_finger_joint1", ctrl_limits: [0.2, -1, 20] },
l_panda_leftfinger(l_panda_finger_joint1): {  },
l_panda_rightfinger(l_panda_finger_joint2): {  },
l_panda_leftfinger_0(l_panda_leftfinger): { shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/finger.ply>, visual: True },
l_panda_rightfinger_0(l_panda_rightfinger): { rel: [0, 0, 0, -1.03412e-13, 0, 0, 1], shape: mesh, mesh: <../models/rai-robotModels/panda/franka_description/meshes/visual/finger.ply>, visual: True },
l_panda_coll1(l_panda_joint1): { rel: [0, 0, -0.15, 1, 0, 0, 0], shape: capsule, size: [0.2, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll3(l_panda_joint3): { rel: [0, 0, -0.15, 1, 0, 0, 0], shape: capsule, size: [0.2, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll5(l_panda_joint5): { rel: [0, 0.02, -0.2, 1, 0, 0, 0], shape: capsule, size: [0.22, 0.09], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll2(l_panda_joint2): { shape: capsule, size: [0.12, 0.12], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll4(l_panda_joint4): { shape: capsule, size: [0.12, 0.08], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll6(l_panda_joint6): { rel: [0, 0, -0.04, 1, 0, 0, 0], shape: capsule, size: [0.1, 0.07], color: [1, 1, 1, 0.2], contact: -2 },
l_panda_coll7(l_panda_joint7): { rel: [0, 0, 0.01, 1, 0, 0, 0], shape: capsule, size: [0.1, 0.07], color: [1, 1, 1, 0.2], contact: -2 },
l_l_gripper(l_panda_joint7): { rel: [-2.69422e-17, 0, 0.22, 2.34326e-17, 0.92388, 0.382683, 5.65713e-17],
shape: marker,
size: [0.0],
color: [0.9, 0.9, 0.9],
logical: { is_gripper: True } },
l_palm(l_panda_hand_joint): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0], shape: capsule, size: [0.14, 0.07], color: [1, 1, 1, 0.2], contact: -3 },
l_finger1(l_panda_finger_joint1): { rel: [0, 0.028, 0.035, 1, 0, 0, 0], shape: capsule, size: [0.02, 0.03], color: [1, 1, 1, 0.2], contact: -2 },
l_finger2(l_panda_finger_joint2): { rel: [0, -0.028, 0.035, 1, 0, 0, 0], shape: capsule, size: [0.02, 0.03], color: [1, 1, 1, 0.2], contact: -2 }
