# general settings

# resize the renderer
render_width                = 200
render_height               = 200

# gltf file input
gltf_path                   = b"/home/user/10M.gltf"

renderer_path               = "/home/user/compound-ray/build/make/lib/libEyeRenderer3.so"

# name of the folder which will be used
#folder_name                = "test_images_" + str(datetime.now()).replace(" ","_")
folder_name                 = "test_images"

# time a frame is visible
min_frame_time_ref          = 0.0
min_frame_time_nav          = 0.0

# agent height above ground
ray_height                  = True
offset_height               = 0.1

# navigate from last to first snapshot
reverse                     = False

# activate debug prints
debug                       = True



# reference settings

# memory snapshots input
csv_input                   = True

# csv files input
read_all_csv_values         = False
read_from_line              = 73   # from 0 to 73 or from 73 to 576 (line-2)
read_until_line             = 576
line_step                   = 1
csv_coor_path               = "/home/user/csv/RouteCoordinates0.csv"
csv_angl_path               = "/home/user/csv/RouteAngles0.csv"
csv_grid_path               = "/home/user/csv/Grid.csv"

# when to store a frame
snapshot_distance           = 0.05



# navigation settings

# navigation mode
bilateral                   = True

# mode 1: fixed step_distance
# mode 2: dynamic to maintain target_speed
# mode 3: based on reference distances
step_distance_mode          = 1
step_distance               = 0.02  # mode 1
target_speed                = 0.35  # mode 2

deg_step                    = 2     # degrees between tested angles

# only compare against window references ahead and look_back references behind
win                         = True
window                      = 30
look_back                   = 10

# frame height to keep of full panorama (ranges from 0 to 1)
start                       = 0.25  # bottom
end                         = 0.75  # top

# # one side degrees mono configuration
search_range                = 30

# bilateral configuration
overlap                     = 270
blind                       = 0
eyeOffset                   = 45
threshold                   = 0.05




# the next settings are rewriting the glTF file specified by the gltf_path string

# render in panoramic mode (necessary for normal eye algorithms), compound settings only relevant when False
panoramic                   = True

# render in compound eye mode, panoramic has to be False
compound_eye                = True
compound_eye_path           = "/home/user/compound-ray/data/eyes/1000-horizontallyAcute-variableDegree.eye" # enum: compound-ray/data/eyes/*.eye files
compound_eye_projection     = "spherical_orientationwise" # enum: compound-ray/data/compound-eye-custom-properties.txt

# the background while rendering, hdri_path is only relevant when background = "hdri"
background                  = "simple_sky" # enum: compound-ray/libEyeRenderer3/shaders.cu (white_background, black_background, simple_sky, hdri)
hdri_path                   = "/home/user/your_background.hdr"

# glTF mesh offset (y is height)
offset_mesh                 = [-17.85, 0, 7]
