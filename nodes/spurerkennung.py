#!/usr/bin/env python3
# coding: utf-8
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
from simulation_image_helper import SimulationImageHelper
import time
import random


helper = SimulationImageHelper()

def LS_lane_fit(pL, pR):
    """
    LS estimate for lane coeffients z=(W, Y_offset, Delta_Phi, c0)^T.

    Args:
        pL: [NL, 2]-array of left marking positions (in DIN70000)
        pR: [NR, 2]-array of right marking positions (in DIN70000)

    Returns:
        Z: lane coeffients (W, Y_offset, Delta_Phi, c0)
    """

    H = np.zeros((pL.shape[0]+pR.shape[0], 4)) # design matrix
    Y = np.zeros((pL.shape[0]+pR.shape[0], 1)) # noisy observations

    # fill H and Y for left line points
    for i in range(pL.shape[0]):
        u, v = pL[i,0], pL[i,1]
        u2 = u*u
        H[i, :] = [0.5, -1, -u, 1.0/2.0 * u2]
        Y[i] = v

    # fill H and Y for right line points
    for i in range(pR.shape[0]):
        u, v = pR[i,0], pR[i,1]
        u2 = u*u
        u3 = u2*u
        H[pL.shape[0]+i, :] = [-0.5, -1, -u, 1.0/2.0 * u2]
        Y[pL.shape[0]+i] = v

    # compute optimal state vector Z
    Z = np.dot(np.linalg.pinv(H), Y)

    return Z


def LS_lane_compute(Z, maxDist=60, step=0.5):
    """
    Compute lane points from given parameter vector.

    Args;
        Z: lane coeffients (W, Y_offset, Delta_Phi, c0)
        maxDist[=60]: distance up to which lane shall be computed
        step[=0.5]: step size in x-direction (in m)

    Returns:
        (x_pred, yl_pred, yr_pred): x- and y-positions of left and
            right lane points
    """
    x_pred = np.arange(0, maxDist, step)
    yl_pred = np.zeros_like(x_pred)
    yr_pred = np.zeros_like(x_pred)

    for i in range(x_pred.shape[0]):
        u = x_pred[i]
        u2 = u*u
        yl_pred[i] = np.dot( np.array([ 0.5, -1, -u, 1.0/2.0 * u2]), Z )
        yr_pred[i] = np.dot( np.array([-0.5, -1, -u, 1.0/2.0 * u2]), Z )

    return (x_pred, yl_pred, yr_pred)


def LS_lane_residuals(lane_left, lane_right, Z):


    ## HIER CODE EINFUEGEN
    H = np.zeros((lane_left.shape[0]+lane_right.shape[0], 4)) # design matrix
    Y = np.zeros((lane_left.shape[0]+lane_right.shape[0], 1))

    for i in range(lane_left.shape[0]):
        u, v = lane_left[i,0], lane_left[i,1]
        u2 = u*u
        H[i, :] = [0.5, -1, -u, 1.0/2.0 * u2]
        Y[i] = v

    for i in range(lane_right.shape[0]):
        u, v = lane_right[i,0], lane_right[i,1]
        u2 = u*u
        u3 = u2*u
        H[lane_left.shape[0]+i, :] = [-0.5, -1, -u, 1.0/2.0 * u2]
        Y[lane_left.shape[0]+i] = v

    residual = np.dot(H, Z) - Y


    ## EIGENER CODE ENDE

    return residual


def LS_lane_inliers(residual, thresh):

    ## HIER CODE EINFUEGEN
    true_inliers = 0

    true_inliers = np.abs( residual ) < thresh
    n_inliners = np.sum(true_inliers)

    return n_inliners


    ## EIGENER CODE ENDE


def draw_points(debug_img,pts,color):
    pts_img=helper.road2image(pts)
    pts_img=np.asarray(pts_img).reshape(-1,2).astype(int)
    for px,py in pts_img:
        if 0<=px<debug_img.shape[1] and 0<= py < debug_img.shape[0]:
            cv2.circle(debug_img,(px,py),2,color,-1)

# test with previous estimate
def spurerkennung(im):
    S = 4
    dist_thresh = 0.4
    max_num_inliers = 0
    Z_opt = None
    num_iters = 50
    #im = cv2.resize(im, (0,0), fx=0.75, fy=0.75)
    #gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(im, 50, 1000)
    y, x = np.where(edges > 0)
    points = np.column_stack((x, y)) 

    lane_left = np.empty((0,2))
    lane_right = np.empty((0,2))
    Z_all = LS_lane_fit(lane_left, lane_right) 
    x_pred, yl_pred, yr_pred = LS_lane_compute(Z_all)
    residual = LS_lane_residuals(lane_left, lane_right, Z_all)

    roadPoints = helper.image2road(points)
    roadPoints = np.asarray(roadPoints).reshape(-1, 2)

    lane_width = 3
    roadPoints = roadPoints[
        (roadPoints[:,1] > -lane_width) &
        (roadPoints[:,1] <  lane_width)
    ]


    max_range_m = 30
    roi_right_line = np.array([
        [0, 0],
        [6, 0],
        [max_range_m, 8],
        [max_range_m, -15],
        [0, -15] ])

    roi_left_line = np.array([
        [0, 0],
        [6, 0],
        [max_range_m, -8],
        [max_range_m, 15],
        [0, 15] ])

    M = roadPoints

    for i in range(M.shape[0]):
        if cv2.pointPolygonTest(roi_left_line, (M[i,0], M[i,1]), False) > 0:
            lane_left = np.vstack((lane_left, M[i,:]))
        if cv2.pointPolygonTest(roi_right_line, (M[i,0], M[i,1]), False) > 0:
            lane_right = np.vstack((lane_right, M[i,:]))

    lane_left_back = helper.road2image(lane_left)
    imagePoints_back = np.asarray(lane_left_back).reshape(-1, 2)

    lane_right_back = helper.road2image(lane_right)
    imagePoints_back = np.asarray(lane_right_back).reshape(-1, 2)

    for i in range(num_iters):
        # draw S//2 random indices for points in lane_left and lane_right, respectively
        idx_left = random.sample(range(lane_left.shape[0]), S//2)
        idx_right = random.sample(range(lane_right.shape[0]), S//2)

        ## HIER CODE EINFUEGEN
        Z_tmp = LS_lane_fit(lane_left[idx_left, :], lane_right[idx_right, :])
        residual = LS_lane_residuals(lane_left, lane_right, Z_tmp)
        inlier_mask = np.abs(residual) < dist_thresh
        num_inliers = LS_lane_inliers(residual, dist_thresh)
        if num_inliers > max_num_inliers:
            max_num_inliers = num_inliers
            Z_opt = Z_tmp


    x_pred, yl_pred, yr_pred = LS_lane_compute(Z_opt)
    debug = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    
    # Punkte transformieren zurück ins Bild
    lane_left_img  = np.asarray(helper.road2image(lane_left)).reshape(-1,2).astype(int)
    lane_right_img = np.asarray(helper.road2image(lane_right)).reshape(-1,2).astype(int)

    # Linke Punkte blau
    for x, y in lane_left_img:
        cv2.circle(debug, (x, y), 2, (255, 0, 0), -1)

    # Rechte Punkte rot
    for x, y in lane_right_img:
        cv2.circle(debug, (x, y), 2, (0, 0, 255), -1)

    # Linien transformieren
    yl_img = np.asarray(helper.road2image(np.column_stack((x_pred,
    yl_pred)))).reshape(-1,2).astype(int)
    yr_img = np.asarray(helper.road2image(np.column_stack((x_pred,
    yr_pred)))).reshape(-1,2).astype(int)

    # Linke Linie grün
    for i in range(len(yl_img)-1):
        cv2.line(debug, tuple(yl_img[i]), tuple(yl_img[i+1]), (0,255,0), 2)

    # Rechte Linie grün
    for i in range(len(yr_img)-1):
        cv2.line(debug, tuple(yr_img[i]), tuple(yr_img[i+1]), (0,255,0), 2)


    return Z_opt, debug
    


