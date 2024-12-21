world: {},

table (world){
    shape:ssBox, Q: "t(0 0. 0)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact: 1
}

ob_plane(table){}

obj_comb(ob_plane): {X: [-0.15 0.0 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/comb.stl>}
at_comb(obj_comb) : {X: [-0.15 -0.025 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_sphere(ob_plane): {X: [-0.15 -0.15 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/sphere.stl>}
at_sphere(obj_sphere) : {X: [-0.15 -0.17 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_prism_long(ob_plane): {X: [-0.05 0.15 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/prism_long.stl>}
at_prism_long(obj_prism_long) : {X: [-0.05 0.25 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_square(ob_plane): {X: [-0.05 -0.1 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/square.stl>}
at_square(obj_square) : {X: [-0.05 -0.125 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_rectangle(ob_plane): {X: [0.05 0.06 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/rectangle.stl>}
at_rectangle(obj_rectangle) : {X: [0.05 -0.01 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_prism_short(ob_plane): {X: [0.05 -0.2 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/prism_short.stl>}
at_prism_short(obj_prism_short) : {X: [0.05 -0.15 0.11, 1 0 0 0], shape: marker, size: [0.1]}

#obj_cube_hollow(ob_plane): {X: [0.15 -0.2 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/cube_hollow.stl>}
#at_cube_hollow(obj_cube_hollow) : {X: [0.15 -0.22 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_cube(ob_plane): {X: [0.15 -0.1 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/cube.stl>}
at_cube(obj_cube) : {X: [0.15 -0.12 0.11, 1 0 0 0], shape: marker, size: [0.1]}

#obj_cylinder_hollow(ob_plane): {X: [0.15 0.2 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/cylinder_hollow.stl>}
#at_cylinder_hollow(obj_cylinder_hollow) : {X: [0.15 0.15 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_cylinder(ob_plane): {X: [0.24 0.0 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/cylinder.stl>}
at_cylinder(obj_cylinder) : {X: [0.24 0.05 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_disk(ob_plane): {X: [0.24 -0.15 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/disk.stl>}
at_disk(obj_disk) : {X: [0.24 -0.18 0.11, 1 0 0 0], shape: marker, size: [0.1]}

#obj_prism_short_hollow(ob_plane): {X: [0.24 0.12 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/prism_short_hollow.stl>}
#at_prism_short_hollow(obj_prism_short_hollow) : {X: [0.19 0.12 0.11 0.7 0 0 0.7], shape: marker, size: [0.1]}

#obj_sphere_half_hollow(ob_plane): {X: [-0.05 -0.2 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/sphere_half_hollow.stl>}
#at_sphere_half_hollow(obj_sphere_half_hollow) : {X: [-0.05 -0.22 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_sphere_half(ob_plane): {X: [-0.15 -0.07 0.11 0.7 0.7 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/sphere_half.stl>}
at_sphere_half(obj_sphere_half) : {X: [-0.15 -0.09 0.11, 1 0 0 0], shape: marker, size: [0.1]}

obj_trapezoid(ob_plane): {X: [-0.15 0.15 0.11 1 0 0 0], joint:rigid, color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../models/simple/trapezoid.stl>}
at_trapezoid(obj_trapezoid) : {X: [-0.15 0.12 0.11, 1 0 0 0], shape: marker, size: [0.1]}









cam_front(world): { rel: [0, 1, 1,  0.00000000,   0.00000000,  -0.92387953,   0.38268344 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_back(world): { rel: [0, -1, 1,    0.38268343,  -0.92387953,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_right(world): { rel: [-1, 0.1, 1,   0.27059805,  -0.65328148,   0.65328148,  -0.27059805 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  },
cam_left(world): { rel: [1, 0.1, 1,   0.27059805,  -0.65328148,  -0.65328148,   0.27059805 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  },
cam_up(world): { rel: [0, 0, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  },
cam_down(world): { rel: [0, 0, -1, 1,  0, 0, 0], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3]  }
cam_front_right(world): { rel: [-0.65, 0.65, 1,   0.17337590,  -0.35808816,   0.85149587,  -0.34156764], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_front_left(world): { rel: [0.75, 0.75, 1,   0.17987373,  -0.37692368,  -0.84218296,   0.34103055 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_back_right(world): { rel: [-0.65, -0.65, 1,   0.34935254,  -0.85318381,   0.35729011,  -0.14957931 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },
cam_back_left(world): { rel: [0.65, -0.65, 1,   0.34606604,  -0.85506437,  -0.35627669,   0.14889642 ], shape: marker, size: [0.1], width: 240, height: 240, focalLength: 3, zRange: [0.5, 3] },


