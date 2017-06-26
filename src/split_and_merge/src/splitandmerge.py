#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np

def angle_wrap(ang):
    """
    Return the angle normalized between [-pi, pi].

    Works with numbers and numpy arrays.

    :param ang: the input angle/s.
    :type ang: float, numpy.ndarray
    :returns: angle normalized between [-pi, pi].
    :rtype: float, numpy.ndarray
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang

def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    

def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1] - 1

    # Check number of points
    if last_pt - first_pt + 1 < min_points:
        return None

    # Line defined as "a*x + b*y + c = 0"
    # Calc (y1-y2)x + (x2-x1)y + (x1y2 - x2y1) = 0
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
    line /= np.linalg.norm(line[:2]) # normalize by || a, b ||

    # Distances
    pts = np.vstack(( points, np.ones((1, points.shape[1])) )).T
    dists = np.dot(pts, line)
    dists = np.abs(dists)
    dmax = np.max(dists[first_pt:last_pt+1])

    # Split
    if dmax > split_thres:
        
        # Check sublines
        idx = np.argmax(dists == dmax) 
        prev = split(points, split_thres, inter_thres, min_points, first_pt, idx)
        post = split(points, split_thres, inter_thres, min_points, idx, last_pt)
        
        # Return results
        if prev is not None and post is not None:
            return np.vstack((prev, post))
        elif prev is not None:
            return prev
        elif post is not None:
            return post
        else:
            return None

    # Do not need to split furthermore
    else:
        # Optional check interpoint distance
        for i in range(first_pt, last_pt):
            x3 = points[0, i]
            y3 = points[1, i]
            x4 = points[0, i+1]
            y4 = points[1, i+1]
            if np.sqrt((x3-x4)**2 + (y3-y4)**2) > inter_thres:
                #Split lines
                prev = split(points, split_thres, inter_thres, min_points, first_pt, i)
                post = split(points, split_thres, inter_thres, min_points, i+1, last_pt)
                
                # Return results
                if prev is not None and post is not None:
                    return np.vstack((prev, post))
                elif prev is not None:
                    return prev
                elif post is not None:
                    return post
                else:
                    return None
        

        return np.array([[x1, y1, x2, y2]])


def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    # No data received
    if lines is None:
        return np.array([])
        
    # Check and merge similar lines
    i = 0
    while i < lines.shape[0] - 1:
        
        # Line angles
        ang1 = np.arctan2(lines[i,3]  -lines[i,1],   lines[i,2]  -lines[i,0])
        ang2 = np.arctan2(lines[i+1,3]-lines[i+1,1], lines[i+1,2]-lines[i+1,0])
        
        # Below thresholds
        ang = abs(angle_wrap(ang1-ang2))
        dis = np.sqrt( (lines[i,2] - lines[i+1,0])**2 + \
                       (lines[i,3] - lines[i+1,1])**2 )
        if ang < ang_thres and dis < dist_thres:
            
            # Joined line
            lines[i,:]=[lines[i,0], lines[i,1], lines[i+1,2], lines[i+1,3]]
            
            # Delete unnecessary line
            lines = np.delete(lines, i+1, axis=0)
            
        # Nothing to merge
        else:
            i += 1
            
    return lines
