import os
import cv2
import sys
import math 
import config
import collections 
import numpy as np 
import matplotlib.pyplot as plt
from mayavi import mlab 
from scipy.linalg import lstsq
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import least_squares


def extract_features(image_names):

    sift = cv2.xfeatures2d.SIFT_create(0,3,0.04,10)
    key_points_for_all = []
    descriptor_for_all = []
    colors_for_all = []

    for image_name in image_names:
        image = cv2.imread(image_name)

        if image is None:
            continue

        key_points, descriptor = sift.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),None)

        if len(key_points) <= 10:
            continue

        key_points_for_all.append(key_points)
        descriptor_for_all.append(descriptor)
        colors = np.zeros((len(key_points), 3))

        for i,key_point in enumerate(key_points):
            p = key_point.pt
            colors[i] = image[int(p[1])][int(p[0])]
        colors_for_all.append(colors)
    return np.array(key_points_for_all) , np.array(descriptor_for_all) , np.array(colors_for_all)

def match_features(query, train):
    bf = cv2.BFMatcher(cv2.NORM_L2)
    knn_matches = bf.knnMatch(query,train, k=2)
    matches = []

    for m, n in knn_matches:
        if m.distance < config.MRT * n.distance:
            matches.append(m)

    return np.array(matches)

def match_all_features(descriptor_for_all):
    matches_for_all = []
    for i in range(len(descriptor_for_all)-1):
        matches = match_features(descriptor_for_all[i], descriptor_for_all[i+1])
        matches_for_all.append(matches)
    return np.array(matches_for_all)

def find_transform(K, p1, p2):
    focal_length = 0.5 * (K[0,0] + K[1, 1])
    principle_point = (K[0, 2], K[1, 2])

    E,mask = cv2.findEssentialMat(p1, p2, focal_length, principle_point, cv2.RANSAC, 0.999, 1.0)
    cameraMatrix = np.array([[focal_length, 0, principle_point[0]], [0, focal_length, principle_point[1]], [0, 0, 1]])
    pass_count, R, T, mask = cv2.recoverPose(E, p1, p2, cameraMatrix, mask)

    return R, T, mask

def get_matched_points(p1, p2, matches):

    src_pts = np.asarray([p1[m.queryIdx].pt for m in matches])
    dst_pts = np.asarray([p2[m.trainIdx].pt for m in matches])

    return src_pts , dst_pts

def get_matched_colors(c1, c2, matches):
    color_src_pts = np.asarray([c1[m.queryIdx] for m in matches])
    color_dst_pts = np.asarray([c2[m.trainIdx] for m in matches])

    return color_src_pts, color_dst_pts

def maskout_points(p1, mask):

    p1_copy = []
    for i in range(len(mask)):
        if mask[i] > 0:
            p1_copy.append(p1[i])

    return np.array(p1_copy)

def init_structure(K, key_points_for_all, colors_for_all, matches_for_all):
    p1, p2 = get_matched_points(key_points_for_all[0], key_points_for_all[1], matches_for_all[0])
    c1, c2 = get_matched_colors(colors_for_all[0], colors_for_all[1], matches_for_all[0])

    if find_transform(K, p1, p2):
        R,T,mask = find_transform(K, p1, p2)
    else
        R,T,mask = np.array([]), np.array([]), np.array([])

    p1 = maskout_points(p1, mask)
    p2 = maskout_points(p2, mask)
    colors = maskout_points(c1, mask)

    R0 = np.eye(3,3)
    T0 = np.zeros((3,1))
    structure = reconstruct(K, R0 , T0, R, T, p1, p2)
    rotations = [R0, R]
    motions = [T0, T]
    correspond_struct_idx = []
    for key_p in key_points_for_all:
        correspond_struct_idx.append(np.ones(len(key_p)) * -1)
    correspond_struct_idx = np.array(correspond_struct_idx)
    idx=0
    matches = matches_for_all[0]
    for i,match in enumerate(matches):
        if mask[i] == 0:
            continue
        correspond_struct_idx[0][int(match.queryIdx)] = idx
        correspond_struct_idx[1][int(match.queryIdx)] = idx
        idx += 1

    return structure, correspond_struct_idx, colors, rotations, motions


def reconstruct(K, R1, T1, R2, T2, p1, p2):

    proj1 = np.zeros((3, 4))
    proj2 = np.zeros((3, 4))
    proj1[0:3, 0:3] = np.float32(R1)
    proj1[:, 3] = np.float32(T1.T)
    proj2[0:3, 0:3] = np.float32(R2)
    proj2[:, 3] = np.float32(T2.T)
    fk = np.float32(K)
    proj1 = np.dot(fk, proj1)
    proj2 = np.dot(fk, proj2)
    s = cv2.triangulatePoints(proj1, proj2, p1.T, p2.T)
    structure = []

    for i in range(len(s[0])):
        col = s[:, i]
        col /= col[3]
        structure.append([col[0], col[1], col[2]])

    return np.array(structure)

def fusion_structure(matches, struct_indices, next_struct_indices, structure, next_structure, colors, next_colors):

    for i,match in enumerate(matches):
        query_idx = match.queryIdx
        train_idx = match.trainIdx
        struct_idx = struct_indices[query_idx]
        if struct_idx >= 0:
            next_struct_indices[train_idx] = struct_idx
            continue
        structure = np.append(structure, [next_structure[i]], axis=0)
        colors = np.append(colors, [next_colors[i]], axis=0)
        struct_indices[query_idx]= next_struct_indices[train_idx] = len(structure) - 1
    return struct_indices, next_struct_indices, structure, colors

def get_objpoints_and_imgpoints(matches, struct_indices, structure, key_points):

    object_points= []
    image_points = []
    for match in matches:
        query_idx = match.queryIdx
        train_idx = match.trainIdx
        struct_idx = struct_indices[query_idx]
        if struct_idx < 0:
            continue
        object_points.append(structure[int(struct_idx)])
        image_points.append(key_points[train_idx].pt)

    return np.array(object_points), np.array(image_points)





