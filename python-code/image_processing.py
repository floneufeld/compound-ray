import config
import numpy as np

from utils import matlab_round

# normalization like in the matlab experiment
def normalize_5pct(x, axis=0):
    x = np.asarray(x, dtype=float)

    xmin = np.min(x, axis=axis, keepdims=True)
    xmax = np.max(x, axis=axis, keepdims=True)

    cutoff = xmin + 0.05 * (xmax - xmin)

    denominator = xmax - cutoff

    # avoid division by zero for a completely flat curve
    denominator = np.where(denominator == 0, 1.0, denominator)

    return (x - cutoff) / denominator

# normalization for pearson correlation
def normalize_for_correlation(img):
    x = img.astype(np.float32)
    x -= x.mean()

    norm = np.linalg.norm(x)

    if norm > 1e-8:
        x /= norm

    return x

# crop and normalize reference snapshots
def process_reference_images(raw_imgs):
    ref_imgs = []
    ref_left = []
    ref_right = []

    for frame in raw_imgs:
        
        rgb         = frame[:, :, :3]                           # remove alpha
        h           = rgb.shape[0]                              # height
        start_px    = int(h * config.start)                     # start at bottom
        end_px      = int(h * config.end)                       # end at top
        cropped     = rgb[start_px:end_px, :, :]                # keep from start to end
        gray        = cropped.mean(axis=2).astype(np.float32)   # mean of rgb
        left, right = split_panorama(gray, config.overlap, config.blind)
        full        = normalize_for_correlation(gray)
        left        = normalize_for_correlation(left)
        right       = normalize_for_correlation(right)
        
        ref_imgs.append(full.ravel())
        ref_left.append(left.ravel())
        ref_right.append(right.ravel())

    return (
        np.asarray(ref_imgs, dtype=np.float32),
        np.asarray(ref_left, dtype=np.float32),
        np.asarray(ref_right, dtype=np.float32)
    )


# splits the panorama into left and right versions for bilateral algorithm
def split_panorama(img, overlap_deg, blind_deg):

    ncols = img.shape[1]
    
    # remove blindspot
    if 0 < blind_deg:
        px_per_deg  = ncols / 360.0
        blind_px    = matlab_round(blind_deg*px_per_deg)
        half_blind  = blind_px / 2.0
        left_cut    = matlab_round(half_blind)
        right_cut   = ncols - matlab_round(half_blind)
        img         = img[:, left_cut:right_cut]
    
    ncols   = img.shape[1]
    centre  = int(ncols / 2.0)

    if 0 < overlap_deg < 360:
        
        px_per_deg      = ncols / (360.0 - blind_deg)
        overlap_px      = overlap_deg * px_per_deg

        half_overlap    = matlab_round(overlap_px / 2.0)
        
        left_end        = int(centre + half_overlap)
        right_start     = int(centre - half_overlap)
        
        left            = img[:, :left_end]
        right           = img[:, right_start:ncols]
        

    elif overlap_deg == 360:

        left = img.copy()
        right = img.copy()

    elif 0 > overlap_deg:

        px_per_deg      = ncols / (360.0 - blind_deg)
        overlap_px      = overlap_deg * px_per_deg

        half_overlap    = matlab_round(abs(overlap_px) / 2.0)

        left_end        = int(centre - half_overlap)
        right_start     = int(centre + half_overlap)

        left            = img[:, :left_end]
        right           = img[:, right_start:ncols]
    
    else:

        left = img[:, :centre]
        right = img[:, centre:ncols]

    return left, right

