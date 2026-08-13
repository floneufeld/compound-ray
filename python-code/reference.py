import time
import math
import numpy as np
import csv
from itertools import islice
from ctypes import *

import config
import constants
from geometry import convert_coor
from renderer import ray_offset, set_pose, render
from utils import wait
from image_processing import process_reference_images


def reference(eye_renderer):
    if config.csv_input:
        ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor = ref_csv(eye_renderer)
        return ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor
    else:
        ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor = ref_fly(eye_renderer)
        return ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor

# read and store route csv values (z-up)
def read_csv():
    
    coor_rows = []
    angl_rows = []

    if config.read_all_csv_values:
        coor_rows = list(csv.DictReader(open(config.csv_coor_path, "r")))
        angl_rows = list(csv.DictReader(open(config.csv_angl_path, "r")))
        #coor_rows = list(csv.DictReader(open(config.csv_grid_path, "r")))
    else:
        with open(config.csv_coor_path, "r") as fc:
            reader_c    = csv.DictReader(fc)
            coor_rows   = list(islice(reader_c, config.read_from_line, config.read_until_line, config.line_step))
            print(len(coor_rows))
        with open(config.csv_angl_path, "r") as fa:
            reader_a    = csv.DictReader(fa)
            angl_rows   = list(islice(reader_a, config.read_from_line, config.read_until_line, config.line_step))
    
    return coor_rows, angl_rows


def ref_csv(eye_renderer):
    
    # store csv data
    coor_rows, angl_rows = read_csv()
    
    # number of reference images -1 because we dont need ref at destination and we will access i+1 in ref loop
    num_refs = len(coor_rows)-1

    raw_imgs        = []
    ref_left_vec    = []
    ref_right_vec   = []
    distances       = []
    
    # loop to create all reference images
    for i in range(num_refs):
        
        # get time for speed adjustment
        start_time_ref = time.time()

        # set current coordinate and look to the next coordinate
        row_coor    = coor_rows[i]      # current coordinate
        next_coor   = coor_rows[i+1]    # next coordinate
        #row_angl   = angl_rows[i]
        #next_angl  = angl_rows[i+1]

        # current xyz in z-up csv
        x = float(row_coor["X"])
        y = float(row_coor["Y"])
        z = float(row_coor["Z"])
        #xa = float(row_angl["X"])
        #ya = float(row_angl["Y"])
        #za = float(row_angl["Z"])

        # next xyz in z-up csv
        nx = float(next_coor["X"])
        ny = float(next_coor["Y"])
        nz = float(next_coor["Z"])
        #nxa = float(next_angl["X"])
        #nya = float(next_angl["Y"])
        #nza = float(next_angl["Z"])

        # render-space: convert z-up -> y-up
        rx,  ry,  rz        = convert_coor(x, y, z)
        rnx, rny, rnz       = convert_coor(nx, ny, nz)
        #rxa,  rya,  rza    = convert_coor(xa, ya, za)
        #rnxa, rnya, rnza   = convert_coor(nxa, nya, nza)

        # distances to next coordinate (render y-up)
        dx = rnx - rx
        dy = rny - ry
        dz = rnz - rz
        horizontal = math.sqrt(dx*dx + dz*dz)
        distances.append(horizontal)

        # rotations
        yaw   = math.atan2(dx, dz) # y-up
        #pitch = math.atan2(dy, horizontal)
        pitch = 0.0 # dont look up or down
        roll  = 0.0 # dont rotate
        
        # height offset calculation
        if config.ray_height: ry = ray_offset(eye_renderer, rx, rz)
        
        # apply calculated coordinate and rotation
        set_pose(eye_renderer, rx, ry, rz, yaw, pitch) # pitch and roll = 0 (optional)

        # render, display and poll events
        render(eye_renderer)

        # save the frame as a .ppm .jpeg .jpg .png file directly from the renderer
        eye_renderer.saveFrameAs(c_char_p((config.folder_name + "/test-imageREF" + str(i) + ".jpeg").encode()))

        # get frame, store it in array and process later
        frame_ptr = eye_renderer.getFramePointer()
        frame = np.ctypeslib.as_array(frame_ptr, shape=(config.render_height, config.render_width, 4))
        raw_imgs.append(frame.copy())

        # DEBUG: coordinates, distances and yaw for all references
        if config.debug:
            print("rx:\t", rx, "\try:\t", ry, "\trz:\t", rz)
            print("rnx:\t", rnx, "\trny:\t", rny, "\trnz:\t", rnz)
            print("dx:\t", dx)
            print("dy:\t", dy)
            print("dz:\t", dz)
            print("yaw:\t", math.degrees(yaw) % 360, "deg")

        # enforce minimum visible time
        wait(start_time_ref, config.min_frame_time_ref)
        print("")
    
    coor_one        = convert_coor(coor_rows[0]["X"], coor_rows[0]["Y"], coor_rows[0]["Z"])
    coor_two        = convert_coor(coor_rows[1]["X"], coor_rows[1]["Y"], coor_rows[1]["Z"])
    sec_last_coor   = convert_coor(coor_rows[-2]["X"], coor_rows[-2]["Y"], coor_rows[-2]["Z"])
    last_coor       = convert_coor(coor_rows[-1]["X"], coor_rows[-1]["Y"], coor_rows[-1]["Z"])
    
    if config.reverse:
        raw_imgs.reverse()
        raw_imgs = np.roll(raw_imgs, raw_imgs[0].shape[1] // 2, axis=2)
        coor_one, coor_two, sec_last_coor, last_coor = last_coor, sec_last_coor, coor_two, coor_one
    
    # now process the frames
    ref_gray_vec, ref_left_vec, ref_right_vec = process_reference_images(raw_imgs)
    
    # store length stats for dynamic step distance
    distances.sort()
    
    # DEBUG: shape
    if config.debug:
        print(f"ref_gray_vec shape: {ref_gray_vec.shape if ref_gray_vec is not None else 'None'}")
        if ref_left_vec.size > 0:
            print(f"left vec shape: {ref_left_vec.shape}")
            print(f"right vec shape: {ref_right_vec.shape}")
        else:
            print("empty")
        print("REFERENCE END\n")
    
    return ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor


# helper for key presses
key_states = {}
def key_pressed_once(eye_renderer, key):
    current = eye_renderer.isKeyPressed(key)

    previous = key_states.get(key, False)
    key_states[key] = current

    return current and not previous


def ref_fly(eye_renderer):
    
    rx = 0.0
    ry = 0.0
    rz = 0.0

    coor_one        = [0.0, 0.0, 0.0]
    coor_two        = [0.0, 0.0, 0.0]
    sec_last_coor   = [0.0, 0.0, 0.0]
    last_coor       = [0.0, 0.0, 0.0]

    yaw     = 0.0
    pitch   = 0.0

    move_speed = 0.01
    turn_speed = 0.01

    i = 0
    
    raw_imgs        = []
    ref_left_vec    = []
    ref_right_vec   = []
    distances       = []

    last_x = 0.0
    last_z = 0.0

    while True:

        # get time for speed adjustment
        start_time_ref = time.time()

        forward_x = math.sin(yaw)
        forward_z = math.cos(yaw)
        right_x = math.cos(yaw)
        right_z = -math.sin(yaw)

        if eye_renderer.isKeyPressed(constants.GLFW_KEY_D):
            rx += right_x * move_speed
            rz += right_z * move_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_A):
            rx -= right_x * move_speed
            rz -= right_z * move_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_W):
            rx += forward_x * move_speed
            rz += forward_z * move_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_S):
            rx -= forward_x * move_speed
            rz -= forward_z * move_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_RIGHT):
            yaw += turn_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_LEFT):
            yaw -= turn_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_UP):
            ry += move_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_DOWN):
            ry -= move_speed
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_PAGE_UP):
            move_speed *= 1.01
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_PAGE_DOWN):
            move_speed /= 1.01
        if key_pressed_once(eye_renderer, constants.GLFW_KEY_SPACE) or (i > 0 and (math.sqrt((rx-last_x)**2 + (rz-last_z)**2) > config.snapshot_distance)):

            # Save the frame as a .ppm .jpeg .jpg .png file directly from the renderer
            eye_renderer.saveFrameAs(c_char_p((config.folder_name + "/test-imageREF" + str(i) + ".jpeg").encode()))

            # get frame, store it in array and process later
            frame_ptr = eye_renderer.getFramePointer()
            frame = np.ctypeslib.as_array(frame_ptr, shape=(config.render_height, config.render_width, 4))
            raw_imgs.append(frame.copy())

            # save distance
            if 0 < i:
                distances.append(math.sqrt((rx-last_x)**2 + (rz-last_z)**2))

            # get coordinates for resetting before navigation
            if 0 == i:
                coor_one[0] = rx
                coor_one[1] = ry
                coor_one[2] = rz
            elif 1 == i:
                coor_two[0] = rx
                coor_two[1] = ry
                coor_two[2] = rz

            # save last position for distance calculation
            last_x = rx
            last_z = rz

            # update last coordinates for destination reached check during navigation and reverse mode
            sec_last_coor[0] = last_coor[0]
            sec_last_coor[1] = last_coor[1]
            sec_last_coor[2] = last_coor[2]
            last_coor[0] = rx
            last_coor[1] = ry
            last_coor[2] = rz

            i += 1

        if eye_renderer.isKeyPressed(constants.GLFW_KEY_ENTER) and (1 < i):
            break

        if eye_renderer.isKeyPressed(constants.GLFW_KEY_ESCAPE):
            raw_imgs.clear()
            distances.clear()
            i       = 0
            rx      = 0.0
            ry      = 0.0
            rz      = 0.0
            yaw     = 0.0
            pitch   = 0.0
        if eye_renderer.isKeyPressed(constants.GLFW_KEY_BACKSPACE):
            raw_imgs.clear()
            distances.clear()
            i = 0

        # height offset calculation
        if config.ray_height:
            ry = ray_offset(eye_renderer, rx, rz)
        
        # apply calculated coordinate and rotation
        set_pose(eye_renderer, rx, ry, rz, yaw, pitch) # pitch and roll = 0 (optional)

        # render, display and poll events
        render(eye_renderer)

        # enforce minimum visible time
        wait(start_time_ref, config.min_frame_time_ref)

    if config.reverse:
        raw_imgs.reverse()
        raw_imgs = np.roll(raw_imgs, raw_imgs[0].shape[1] // 2, axis=2)
        coor_one, coor_two, sec_last_coor, last_coor = last_coor, sec_last_coor, coor_two, coor_one

    # now process the frames
    ref_gray_vec, ref_left_vec, ref_right_vec = process_reference_images(raw_imgs)

    # store length stats for dynamic step distance mode 3
    distances.sort()
    
    if config.debug: print("REFERENCE END\n")
    
    return ref_gray_vec, ref_left_vec, ref_right_vec, distances, coor_one, coor_two, last_coor