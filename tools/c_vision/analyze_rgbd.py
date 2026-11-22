"""Inspect planar height of a synchronized D435 scene relative to the floor."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


root = Path(__file__).resolve().parent
depth = np.load(root / "rgbd_depth.npy").astype(np.float32) / 1000.0
info = json.loads((root / "rgbd_info.json").read_text(encoding="utf-8"))
fx, fy, cx, cy = info["K"][0], info["K"][4], info["K"][2], info["K"][5]
height, width = depth.shape
v, u = np.mgrid[:height, :width]
x = (u - cx) * depth / fx
y = (v - cy) * depth / fy
z = depth
valid = np.isfinite(z) & (z > 0.2) & (z < 1.5)
floor_roi = ((u < 85) | (u > 590) | (v < 30)) & (v < 420)
floor_sample = valid & floor_roi
xyz = np.stack([x[floor_sample], y[floor_sample], z[floor_sample]], axis=1)
for _ in range(5):
    centroid = np.median(xyz, axis=0)
    _, _, vh = np.linalg.svd(xyz - centroid, full_matrices=False)
    normal = vh[-1]
    residual = (xyz - centroid) @ normal
    xyz = xyz[np.abs(residual) < max(0.006, np.quantile(np.abs(residual), 0.8))]
centroid = xyz.mean(axis=0)
_, _, vh = np.linalg.svd(xyz - centroid, full_matrices=False)
normal = vh[-1]
if normal[2] < 0:
    normal = -normal
floor_dist = (np.stack([x, y, z], axis=2) - centroid) @ normal
floor_dist[~valid] = np.nan
print("floor plane normal", normal, "sample count", len(xyz))
print("background residual mm", np.nanpercentile(floor_dist[floor_sample], [5, 50, 95]) * 1000)
print("scene height distribution mm", np.nanpercentile(floor_dist[valid], [5, 25, 50, 75, 95]) * 1000)

def save_map(values, name, lo, hi):
    scaled = np.clip((values - lo) / (hi - lo), 0, 1)
    rgba = plt.get_cmap("turbo")(scaled)
    rgba[~np.isfinite(values)] = [0, 0, 0, 1]
    Image.fromarray(np.uint8(rgba[:, :, :3] * 255)).save(root / name)

save_map(depth, "rgbd_depth_vis.png", 0.3, 1.1)
save_map(-floor_dist, "rgbd_height_vis.png", -0.02, 0.25)
np.save(root / "rgbd_height.npy", -floor_dist)
