import copy
import numpy as np
import scipy.ndimage as nd
import cv2
from . import cython_min_cost_path as cm
# from numba_check_line_path import numba_check_line_path
# from numba_sample_ridge import numba_sample_ridge, numba_sample_ridge_list
from . import segment_max_height_image as sm
import hello_helpers.hello_misc as hm

def draw_robot_footprint_rectangle(x_pix, y_pix, ang_rad, m_per_pix, image, verbose=False, value=255):
    # One issue to consider is what to do when the mobile base is
    # underneath a surface, such as a table. In these situations,
    # calling this function will erase part of the surface and replace
    # it with floor. This can be problematic, since some robot actions
    # such as stowing or lifting the arm can collide with the surface
    # when mistaking it for floor.
    
    # Robot measurements for rectangular approximation. These should
    # be updated if the robot's footprint changes.
    safety_border_m = 0.02

    # stretch's arm can extend from the side, which adds width to footprint.
    # 0.335 m for the mobile base
    # 0.04 m for the wrist extending over the edge
    # total = 0.335 + (2 * 0.04) due to enforcing symmetry around point of rotation
    robot_width_m = 0.415

    # larger, more conservative model when tethered
    # 0.32 m for the length of the mobile base
    # 0.01 m for the arm E-chain cartridge extending off the back
    # 0.2 m to ignore cables when tethered
    # 0.52 = 0.32 + 0.2
    #robot_length_m = 0.52
    
    # smaller, more optimistic model when untethered
    # 0.32 m for the length of the mobile base
    # 0.01 m for the arm E-chain cartridge extending off the back
    # 0.2 m to ignore cables when tethered
    # 0.33 = 0.32 + 0.01
    robot_length_m = 0.33
    
    origin_distance_from_front_pix_m = 0.05
    
    robot_width_pix = (robot_width_m + (2.0 * safety_border_m))/m_per_pix
    robot_length_pix = (robot_length_m + (2.0 * safety_border_m))/m_per_pix
    origin_distance_from_front_pix = (origin_distance_from_front_pix_m + safety_border_m)/m_per_pix

    # vector to left side of robot
    ls_ang_rad = ang_rad + (np.pi/2.0)
    ls_x = (robot_width_pix/2.0) * np.cos(ls_ang_rad)
    ls_y = -(robot_width_pix/2.0) * np.sin(ls_ang_rad)

    # vector to front of robot
    f_x = origin_distance_from_front_pix * np.cos(ang_rad)
    f_y = -origin_distance_from_front_pix * np.sin(ang_rad)

    # vector to back of robot
    dist_to_back = robot_length_pix - origin_distance_from_front_pix
    b_x = -dist_to_back * np.cos(ang_rad)
    b_y = dist_to_back * np.sin(ang_rad)

    # front left corner of the robot
    fl_x = x_pix + f_x + ls_x
    fl_y = y_pix + f_y + ls_y

    # front right corner of the robot
    fr_x = (x_pix + f_x) - ls_x
    fr_y = (y_pix + f_y) - ls_y
    
    # back left corner of the robot
    bl_x = x_pix + b_x + ls_x
    bl_y = y_pix + b_y + ls_y

    # back right corner of the robot
    br_x = (x_pix + b_x) - ls_x
    br_y = (y_pix + b_y) - ls_y

    corners = np.array([[fl_x, fl_y], [fr_x, fr_y], [br_x, br_y], [bl_x, bl_y]])
    if verbose: 
        print('corners =', corners)
        
    poly_points = np.array(corners)
    poly_points = np.round(poly_points).astype(np.int32)
    
    if image is not None:
        cv2.fillConvexPoly(image, poly_points, value)

def distance_map_simple( floor_mask, m_per_pix, min_robot_width_m,
                         robot_x_pix, robot_y_pix, robot_ang_rad,
                         disallow_too_narrow=True,
                         display_on=False, verbose=False ):
    
    # min_robot_width_m : The best case minimum width of the robot in meters when moving forward and backward. 
    traversable_mask = floor_mask

    # model the robot's footprint as being traversable
    draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, m_per_pix, traversable_mask)
    footprint_test_image = np.zeros_like(traversable_mask)
    draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, m_per_pix, footprint_test_image)
    if display_on:
        cv2.imshow('robot footprint drawing', footprint_test_image)
        cv2.imshow('floor mask after drawing robot footprint', traversable_mask)
    
    # Optimistic estimate of robot width. Use ceil to account for
    # possible quantization. Consider adding a pixel, also.
    min_robot_radius_pix = np.ceil((min_robot_width_m/2.0) / m_per_pix)

    # Fill in small non-floor regions (likely noise)
    #
    # TODO: improve this. For example, fill in isolated pixels that
    # are too small to trust as obstacles. Options include a hit or
    # miss filter, matched filter, or speckle filter.
    fill_in = True
    if fill_in:
        kernel = np.ones((3,3), np.uint8)
        traversable_mask = cv2.morphologyEx(traversable_mask, cv2.MORPH_CLOSE, kernel)
        if display_on:
            cv2.imshow('traversable_mask after filling', traversable_mask)

    # ERROR? : The floodfill operation should occur after removing
    # filtering candidate robot poses due to the footprint
    # radius. Right? Was that too aggressive in the past, so it got
    # dropped?
            
    # Select the connected component of the floor on which the robot
    # is located.
    h, w = traversable_mask.shape
    new_traversable_mask = np.zeros((h+2, w+2), np.uint8)
    #possibly add to floodFill in the future: flags = cv2.FLOODFILL_FIXED_RANGE
    cv2.floodFill(traversable_mask, new_traversable_mask, (robot_x_pix, robot_y_pix), 255)
    traversable_mask = 255 * new_traversable_mask[1:-1,1:-1]
            
    # In previous versions, the traversability mask has treated
    # unobserved pixels and observed non-floor pixels
    # differently. Such as by treating unobserved pixels
    # optimistically as traversable.
            
    # compute distance map: distance from untraversable regions
    #
    # cv2.DIST_L2 : Euclidean distance
    #
    # 5 is the mask size : "finds the shortest path to the nearest
    # zero pixel consisting of basic shifts: horizontal, vertical,
    # diagonal, or knight's move (the latest is available for a 5x5
    # mask)" - OpenCV documentation
    distance_map = cv2.distanceTransform(traversable_mask, cv2.DIST_L2, 5)
    if display_on:
        norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)        
        cv2.imshow('distance map without threshold for the robot width', norm_dist_transform)

    # Restricts the maximum distance of the distance_map. This will
    # favor shorter paths (i.e., straight line paths) when a corridor
    # is wide enough instead of moving to the middle of the
    # corridor. When the corridor is narrower than the threshold, the
    # robot will prefer paths that move it to the center of the
    # corridor. However, simple path planning via 4 connected grid and
    # Dijkstra's algorithm results in vertical and horizontal motions
    # in flat regions rather than point-to-point straight lines.
    clip_max_distance = False
    if clip_max_distance:
        max_distance = 3.0 * min_robot_radius_pix
        print('max_distance =', max_distance)
        print('np.max(distance_map) =', np.max(distance_map))
        # should perform in place clipping
        np.clip(distance_map, None, max_distance, distance_map)
        print('after clipping np.max(distance_map) =', np.max(distance_map))
        if display_on:
            norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)        
            cv2.imshow('distance map with clipped maximum distance', norm_dist_transform)

    if disallow_too_narrow: 
        # set parts of the distance transform that represent free space
        # less than the robot would require to zero
        distance_map[distance_map < min_robot_radius_pix] = 0
        if display_on: 
            norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            cv2.imshow('distance map with robot width threshold', norm_dist_transform)

    # traversable_mask is a binary image that estimates where the
    # robot can navigate given the robot's current pose and the map,
    # but ignoring the robot's radius.

    # distance_map is a scalar image that estimates the distance to
    # the boundaries of the traversable mask.
    return distance_map, traversable_mask