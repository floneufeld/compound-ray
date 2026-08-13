from ctypes import *
import math
import time
import numpy as np

import config
from utils import matlab_round, wait
from geometry import reset
from renderer import ray_offset, set_pose, render
from image_processing import normalize_for_correlation, normalize_5pct, split_panorama


def navigation(coor_0, coor_1, coor_last, eye_renderer, ref_gray_vec, ref_left_vec, ref_right_vec, distances):

    # get first route position and orient toward second position
    rx, ry, rz, current_yaw = reset(coor_0, coor_1)
    #current_yaw = 0.0
    
    # height offset calculation
    if config.ray_height: ry = ray_offset(eye_renderer, rx, rz)
    
    # apply calculated coordinate and rotation
    set_pose(eye_renderer, rx, ry, rz, current_yaw) # pitch and roll = 0 (optional)

    # render, display and poll events
    render(eye_renderer)
    

    # prepare target
    target_rx, target_rz    = float(coor_last[0]), float(coor_last[2])
    distance_to_target      = math.inf
    target_yaw              = 0.0

    
    # counter for navigation steps
    nav_step                = 0

    # estimated memory index
    current_ref             = 0

    # needed for target speed
    last_time               = time.perf_counter()


    # loop until the position is near the destination
    while 0.5 < distance_to_target:
        
        # 1st time measure
        begin_0 = time.perf_counter()

        # DEBUG: coordinate before step
        if config.debug:
            print("Position BEFORE NAV-" + str(nav_step) + ":")
            print("rx:\t", round(rx, 5))
            print("ry:\t", round(ry, 5))
            print("rz:\t", round(rz, 5))


        # get time for minimum frame time visibility
        start_time_nav = time.time()

        # get current frame
        frame_ptr   = eye_renderer.getFramePointer()
        frame       = np.ctypeslib.as_array(frame_ptr, shape=(config.render_height, config.render_width, 4))
        
        # process current frame like in reference section
        rgb         = frame[:, :, :3]
        h           = rgb.shape[0]
        start_px    = int(h * config.start)
        end_px      = int(h * config.end)
        cropped     = rgb[start_px:end_px, :, :]
        gray        = cropped.mean(axis=2).astype(np.float32)
        cur_frame_gray_mean_norm = normalize_for_correlation(gray)  # only for search_mono
        

        # search for the the best heading direction
        beforeSearch_1  = time.perf_counter()
        if not config.bilateral:
            target_yaw, current_ref, score  = search_mono(cur_frame_gray_mean_norm, current_yaw, current_ref, ref_gray_vec)
        else:
            target_yaw, current_ref         = search_bilateral(gray, current_yaw, current_ref, ref_left_vec, ref_right_vec)
        afterSearch_2   = time.perf_counter()
        

        # instant rotation
        current_yaw = target_yaw


        # how far to jump
        # 1: fixed step_distance
        # 2: dynamic to maintain speed
        # 3: based on reference distances
        mode            = config.step_distance_mode
        current_time    = time.perf_counter()
        dt              = current_time - last_time
        last_time       = current_time
        if 1 == mode:
            step_distance   = config.step_distance
        elif 2 == mode:
            step_distance   = config.target_speed * dt
        elif (3 == mode) and not config.bilateral:
            scaled_score    = (score - 0.8) / (1.0 - 0.8)
            scaled_score    = max(0.0, min(1.0, scaled_score))
            step_distance   = distances[int(score * (len(distances)-1))] * 0.5
        else:
            step_distance   = config.step_distance
        

        # calculate new coordinate
        rx += step_distance * math.sin(current_yaw)
        rz += step_distance * math.cos(current_yaw)

        # height offset calculation
        if config.ray_height:
            beforeHeight_3  = time.perf_counter()
            ry              = ray_offset(eye_renderer, rx, rz)
            afterHeight_4   = time.perf_counter()


        afterHeight_5 = time.perf_counter()


        # jump to calculated coordinate and rotation
        set_pose(eye_renderer, rx, ry, rz, current_yaw) # pitch and roll = 0 (optional)

        # render, display and poll events
        render(eye_renderer)

        afterRender_6 = time.perf_counter()

        # Save the frame as a .ppm .jpeg .jpg .png file directly from the renderer
        eye_renderer.saveFrameAs(c_char_p((config.folder_name + "/test-imageNAV" + str(nav_step) + ".jpeg").encode()))

        # update distance to target in 2D
        distance_to_target = math.sqrt((rx - target_rx)**2 + (rz - target_rz)**2)

        # enforce frame minimum visible time
        wait(start_time_nav, config.min_frame_time_nav)

        # last measure
        endTime = time.perf_counter()
        

        # DEBUG
        if config.debug:
            if not config.bilateral: print("score:\t\t\t", round(score*100, 2),   "%")
            print("dt until search:\t",         round((beforeSearch_1-begin_0)*1000, 3),        "ms")
            print("dt search:\t\t",             round((afterSearch_2-beforeSearch_1)*1000, 3),  "ms")
            if config.ray_height: print("dt until height:\t", round((beforeHeight_3-afterSearch_2)*1000, 3), "ms")
            if config.ray_height: print("dt height:\t\t", round((afterHeight_4-beforeHeight_3)*1000, 3), "ms")
            print("dt after height:\t",         round((afterHeight_5-afterSearch_2)*1000, 3),   "ms")
            print("dt after render:\t",         round((afterRender_6-afterHeight_5)*1000, 3),   "ms")
            print("dt from render:\t\t",        round((endTime-afterRender_6)*1000, 3),         "ms")
            print("dt ALL:\t\t\t",              round((endTime-begin_0)*1000, 3),               "ms")
            print("step_distance:\t",           step_distance)
            print("current_yaw:\t",             round(math.degrees(current_yaw) % 360, 5),      "deg")
            print("Position AFTER NAV-" + str(nav_step) + ":")
            print("rx:\t", round(rx, 5))
            print("ry:\t", round(ry, 5))
            print("rz:\t", round(rz, 5))


        # prepare for next iteration
        nav_step += 1
        print("")


# panorama search
def search_mono(cur_frame_fray_mean_norm, current_yaw, current_ref, ref_gray_vec):
    
    # 1st time measure
    begin_0     = time.perf_counter()
    

    # degrees to search
    all_degrees = np.arange(0, 360, config.deg_step)
    search_fov  = min(180, config.search_range) // config.deg_step
    indices     = np.concatenate(
        (np.arange(search_fov), 
        np.arange(len(all_degrees) - search_fov, len(all_degrees)))
        )
        
    
    # window of reference images to compare with
    if config.win:
        start = max(0, current_ref - config.look_back)
        end = min(len(ref_gray_vec), current_ref + config.window)
        ref_vecs = ref_gray_vec[start:end]
    else:
        start = 0
        ref_vecs = ref_gray_vec



    # precompute shifts
    shifts = np.round(cur_frame_fray_mean_norm.shape[1] * all_degrees[indices] / 360).astype(int)

    # create all shifted images
    shiftedFrames = np.empty((cur_frame_fray_mean_norm.size, len(indices)), dtype=np.float32)
    for j, shift in enumerate(shifts):
        shiftedFrames[:, j] = np.roll(cur_frame_fray_mean_norm, -shift, axis=1).ravel()


    beforeMatrix_1 = time.perf_counter()

    # pearson correlation coefficient matrix of reference matrix and angles matrix
    scores = ref_vecs @ shiftedFrames

    afterMatrix_2 = time.perf_counter()

    
    # best score for each tested angle
    local_best = np.argmax(scores, axis=0)

    # arrays to store best results
    best_ref_for_angle      = np.zeros(len(all_degrees), dtype=int)
    best_score_for_angle    = np.zeros(len(all_degrees))

    best_ref_for_angle[indices] = start + local_best
    best_score_for_angle[indices] = (
        scores[local_best, np.arange(len(indices))]
    )

    # index of max value of all max values (from all rotations)
    best_idx = np.argmax(best_score_for_angle)  

    # relative to current direction in degree
    best_angle = all_degrees[best_idx]

    # apply the relative angle turn
    relative_turn = best_angle if best_angle <= 180 else best_angle - 360


    # return 1 of 3: update the global yaw
    target_yaw = current_yaw + math.radians(relative_turn)

    # return 2 of 3: estimated memory index
    ref_before = current_ref
    current_ref = max(current_ref, best_ref_for_angle[best_idx])
    if not config.win: current_ref = -1

    # return 3 of 3: absolute best score
    max_score = best_score_for_angle[best_idx]

    

    # last measure
    endTime = time.perf_counter()


    # DEBUG
    if config.debug:
        print("\tdt until Matrix:\t",   round((beforeMatrix_1-begin_0)      *1000, 3),  "ms")
        print("\tdt MATRIX:\t\t",       round((afterMatrix_2-beforeMatrix_1)*1000, 3),  "ms")
        print("\tdt after Matrix:\t",   round((endTime-afterMatrix_2)       *1000, 3),  "ms")
        if config.win:
            print("\tref before:\t",    ref_before)
            print("\tref after:\t",     current_ref)
            print("\tbest_ref:\t",      best_ref_for_angle[best_idx])
        print("\tdegrees to search:\t", all_degrees[indices])
        print("\tnum of degrees:\t",    len(all_degrees[indices]))
        print("\tbest_angle\t",         best_angle)
        print("\tyaw raw:\t",           round(math.degrees(current_yaw), 5),            "deg")
        print("\tyaw mod:\t",           round(math.degrees(current_yaw) % 360, 5),      "deg")

    return target_yaw, current_ref, max_score



