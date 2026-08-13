import os.path
from ctypes import *

import config
from renderer import create_renderer, configure_gltf
from reference import reference
from navigation import navigation


try:
    # makes sure we have a "test_images" folder
    if not os.path.exists(config.folder_name): os.mkdir(config.folder_name)
    
    # apply settings
    eye_renderer = create_renderer()
    configure_gltf(config.gltf_path)
    
    # Load a scene/glTF/environment
    eye_renderer.loadGlTFscene(c_char_p(config.gltf_path))
    print("HERE")
    # render and save reference frames
    ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor = reference(eye_renderer)

    # follow the route
    navigation(coor_one, coor_two, last_coor, eye_renderer, ref_gray_vec, ref_left_vec, ref_right_vec, distances)

    print("Destination reached successfully!")

    # finally, stop the eye renderer
    eye_renderer.stop()

except Exception as e:
    print(e);
