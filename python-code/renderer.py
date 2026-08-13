from ctypes import *
import eyeRendererHelperFunctions as eyeTools
from numpy.ctypeslib import ndpointer

import json

import config

# apply general settings (folder, renderer, glTF)
def create_renderer():
    
    # load the renderer
    eye_renderer = CDLL(config.renderer_path)
    print("Successfully loaded ", eye_renderer)

    # configure the renderer's function outputs and inputs using the helper functions
    eyeTools.configureFunctions(eye_renderer)

    # resize the renderer display
    # this can be done at any time, but restype of getFramePointer must also be updated to match as such:
    eye_renderer.setRenderSize(config.render_width, config.render_height)
    eye_renderer.getFramePointer.restype = ndpointer(dtype=c_ubyte, shape = (config.render_height, config.render_width, 4))
    # an alternative to the above two lines would be to run:
    #eyeTools.setRenderSize(eye_renderer, config.render_width, config.render_height)

    return eye_renderer

# apply the settings for the glTF file (background, panoramic, compound, mesh offset)
def configure_gltf(gltf_path):
    # store glTF data
    with open(gltf_path, 'r') as file:
        data = json.load(file)

    scene_extras    = data["scenes"][0].setdefault("extras", {})
    camera_extras   = data["cameras"][0].setdefault("extras", {})

    # mesh offset
    mesh_node = next(
        (node for node in data["nodes"] if "mesh" in node),
        None
    )
    if mesh_node is None:
        raise ValueError("No mesh node found in glTF.")
    mesh_node["translation"] = config.offset_mesh
    
    # background
    if "extras" not in data["scenes"][0]:
        data["scenes"][0]["extras"] = {}
    scene_extras["background-shader"] = config.background
    if config.background == "hdri":
        scene_extras["background-hdri"] = config.hdri_path

    # panoramic bool is a string in glTF file
    if config.panoramic:
        camera_extras["panoramic"] = "true"
    else:
        camera_extras["panoramic"] = "false"

    # compound eye rendering
    if config.compound_eye:
        camera_extras["compound-eye"]         = "true"
        camera_extras["compound-structure"]   = config.compound_eye_path
        camera_extras["compound-projection"]  = config.compound_eye_projection
    else:
        camera_extras["compound-eye"]         = "false"

    # write modified data back to glTF file
    with open(gltf_path, 'w') as file:
        json.dump(data, file, indent=4)

# render, display and process events
def render(renderer):
    renderer.renderFrame()
    renderer.displayFrame()
    renderer.processEvents()

# set the cameras pose
def set_pose(renderer, x, y, z, yaw, pitch=0.0, roll=0.0):
    renderer.setCameraPose(
        c_float(x),
        c_float(y),
        c_float(z),
        c_float(pitch),
        c_float(yaw),
        c_float(roll),
    )

# get the height of the ground with an offset
def ray_offset(renderer, x, z):
    hit     = eyeTools.c_float3()
    normal  = eyeTools.c_float3()
    
    ok = renderer.raycastGeometry(
        x, 10.0, z,
        0.0, -1.0, 0.0,
        byref(hit),
        byref(normal))
    
    if ok:
        return hit.y + config.offset_height
    
    return None