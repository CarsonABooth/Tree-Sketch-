import open3d as o3d
import numpy as np
import os
import sys

#Import a single tree point cloud
pcd = o3d.io.read_point_cloud(
    "C:/Users/booth/Open3D/Data/DATA_clouds_ply/wytham_winter_5a.ply"
    )

#Voxel downsampling to remove noise and make processing efficient

downpcd = pcd.voxel_down_sample(voxel_size = .05)

#Estimate the normals for that point cloud
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30)
        )
downpcd.orient_normals_consistent_tangent_plane(k=15)

#o3d.visualization.draw_geometries([downpcd])

#Compute the median point cloud distance for ball pivoting algorithm
#  
distances = downpcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 6 * avg_dist

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(downpcd,o3d.utility.DoubleVector([radius, radius * 2]))

dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

o3d.visualization.draw_geometries([dec_mesh])