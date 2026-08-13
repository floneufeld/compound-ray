import time
import numpy as np

# rounding like matlab
def matlab_round(x):
    return int(np.floor(x + 0.5))

# wait for the specified time if there is time left
def wait(start_time, min_frame_time):
    elapsed     = time.time() - start_time
    remaining   = min_frame_time - elapsed
    if remaining > 0:
        time.sleep(remaining)